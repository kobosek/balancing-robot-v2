#pragma once

#include <cstdint>
#include <mutex>

// A short-lived, process-wide exclusion point for operations that must not
// race with motor arming or persistent configuration writes.  The gate does
// not own hardware and never spans an I/O mutex.  Owners keep the returned
// token for the duration of an operation and release it explicitly.
enum class ControlOperationKind : uint8_t {
    NONE = 0,
    MOTION,
    CONFIGURATION,
    OTA,
    CALIBRATION
};

enum class ControlOperationPhase : uint8_t {
    IDLE = 0,
    RESERVED,
    PREPARING,
    RUNNING,
    WRITING,
    APPLYING,
    SUCCEEDED,
    FAILED,
    ABORTED
};

struct ControlOperationStatus {
    bool known = false;
    bool active = false;
    ControlOperationKind kind = ControlOperationKind::NONE;
    uint64_t operationId = 0;
    ControlOperationPhase phase = ControlOperationPhase::IDLE;
    int32_t resultCode = 0;
    // Configuration transactions expose the document revision that was read
    // and the revision they intended to publish. Other operation kinds leave
    // these fields at zero.
    uint32_t baseRevision = 0;
    uint32_t targetRevision = 0;
};

struct ControlOperationReservation {
    ControlOperationKind kind = ControlOperationKind::NONE;
    uint64_t token = 0;
    // This identifier is exposed to callers. `token` remains an internal
    // ownership proof so a caller cannot release another operation merely by
    // guessing a client supplied operationId.
    uint64_t operationId = 0;

    bool valid() const noexcept
    {
        return kind != ControlOperationKind::NONE && token != 0 &&
            operationId != 0;
    }

    void clear() noexcept
    {
        kind = ControlOperationKind::NONE;
        token = 0;
        operationId = 0;
    }
};

class ControlOperationGate {
public:
    // Non-blocking with respect to other operations: a caller either gets the
    // reservation immediately or must reject/defer its request.  This keeps a
    // state transition from waiting while it owns the StateManager mutex.
    bool tryAcquire(ControlOperationKind kind,
                    ControlOperationReservation& reservation,
                    uint64_t requestedOperationId = 0) noexcept
    {
        if (kind == ControlOperationKind::NONE) {
            return false;
        }

        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_current.valid()) {
            return false;
        }
        // A journaled configuration operation interrupted by reset is the
        // only operation that may proceed until its owner reconciles it.
        // Keeping this policy in the shared gate prevents OTA/calibration or
        // another motion owner from erasing the evidence needed for recovery.
        if (m_recoveryPending && kind != ControlOperationKind::CONFIGURATION) {
            return false;
        }
        // A partially installed OTA bundle has its own recovery path.  An
        // explicit OTA operation may repair it, while motion and calibration
        // remain inhibited until a complete image has booted.
        if (m_otaRecoveryPending &&
            kind != ControlOperationKind::OTA &&
            kind != ControlOperationKind::CONFIGURATION) {
            return false;
        }

        ++m_nextToken;
        if (m_nextToken == 0) {
            ++m_nextToken;
        }
        m_current.kind = kind;
        m_current.token = m_nextToken;
        m_current.operationId = requestedOperationId != 0
            ? requestedOperationId : m_nextToken;
        reservation = m_current;
        m_currentStatus = {
            true,
            true,
            kind,
            m_current.operationId,
            ControlOperationPhase::RESERVED,
            0
        };
        return true;
    }

    bool updatePhase(const ControlOperationReservation& reservation,
                     ControlOperationPhase phase,
                     int32_t resultCode = 0) noexcept
    {
        if (!reservation.valid()) {
            return false;
        }

        std::lock_guard<std::mutex> lock(m_mutex);
        if (!matchesLocked(reservation)) {
            return false;
        }
        m_currentStatus.phase = phase;
        m_currentStatus.resultCode = resultCode;
        return true;
    }

    bool release(ControlOperationReservation& reservation,
                 ControlOperationPhase finalPhase = ControlOperationPhase::SUCCEEDED,
                 int32_t resultCode = 0) noexcept
    {
        if (!reservation.valid()) {
            return false;
        }

        std::lock_guard<std::mutex> lock(m_mutex);
        if (!matchesLocked(reservation)) {
            return false;
        }

        m_currentStatus.active = false;
        m_currentStatus.phase = finalPhase;
        m_currentStatus.resultCode = resultCode;
        m_lastStatus = m_currentStatus;
        m_current.clear();
        m_currentStatus = {};
        reservation.clear();
        return true;
    }

    bool isHeld() const noexcept
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_current.valid();
    }

    bool isHeldBy(ControlOperationKind kind) const noexcept
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_current.valid() && m_current.kind == kind;
    }

    ControlOperationKind currentKind() const noexcept
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_current.kind;
    }

    ControlOperationStatus currentStatus() const noexcept
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_currentStatus;
    }

    ControlOperationStatus lastStatus() const noexcept
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_lastStatus;
    }

    // A journaled configuration operation that was interrupted by a reset
    // keeps motor-active modes inhibited until the configuration owner has
    // explicitly reconciled it. This flag is separate from the live token:
    // no stale mutex owner is fabricated after reboot.
    void setRecoveryPending(bool pending) noexcept
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_recoveryPending = pending;
    }

    bool recoveryPending() const noexcept
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_recoveryPending || m_otaRecoveryPending;
    }

    bool configurationRecoveryPending() const noexcept
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_recoveryPending;
    }

    void setOtaRecoveryPending(bool pending) noexcept
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_otaRecoveryPending = pending;
    }

    bool otaRecoveryPending() const noexcept
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_otaRecoveryPending;
    }

private:
    bool matchesLocked(const ControlOperationReservation& reservation) const noexcept
    {
        return m_current.valid() &&
            m_current.kind == reservation.kind &&
            m_current.token == reservation.token &&
            m_current.operationId == reservation.operationId;
    }

    mutable std::mutex m_mutex;
    ControlOperationReservation m_current;
    uint64_t m_nextToken = 0;
    ControlOperationStatus m_currentStatus;
    ControlOperationStatus m_lastStatus;
    bool m_recoveryPending = false;
    bool m_otaRecoveryPending = false;
};
