#include "unity.h"
#include "ControlOperationGate.hpp"
#include "esp_err.h"

TEST_CASE("control operation gate serializes owners and validates release tokens",
          "[control][gate]")
{
    ControlOperationGate gate;
    ControlOperationReservation motion;
    ControlOperationReservation configuration;

    TEST_ASSERT_TRUE(gate.tryAcquire(ControlOperationKind::MOTION, motion));
    TEST_ASSERT_TRUE(motion.valid());
    TEST_ASSERT_TRUE(gate.isHeld());
    TEST_ASSERT_TRUE(gate.isHeldBy(ControlOperationKind::MOTION));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlOperationKind::MOTION),
                          static_cast<int>(gate.currentKind()));

    TEST_ASSERT_FALSE(gate.tryAcquire(ControlOperationKind::CONFIGURATION,
                                      configuration));
    TEST_ASSERT_FALSE(configuration.valid());

    ControlOperationReservation wrongToken = motion;
    ++wrongToken.token;
    TEST_ASSERT_FALSE(gate.release(wrongToken));
    TEST_ASSERT_TRUE(gate.isHeldBy(ControlOperationKind::MOTION));

    TEST_ASSERT_TRUE(gate.release(motion));
    TEST_ASSERT_FALSE(motion.valid());
    TEST_ASSERT_FALSE(gate.isHeld());

    TEST_ASSERT_TRUE(gate.tryAcquire(ControlOperationKind::CONFIGURATION,
                                     configuration));
    TEST_ASSERT_TRUE(gate.isHeldBy(ControlOperationKind::CONFIGURATION));
    TEST_ASSERT_FALSE(gate.release(wrongToken));
    TEST_ASSERT_TRUE(gate.release(configuration));
    TEST_ASSERT_FALSE(gate.isHeld());
}

TEST_CASE("control operation gate rejects an unowned operation kind",
          "[control][gate]")
{
    ControlOperationGate gate;
    ControlOperationReservation reservation;
    TEST_ASSERT_FALSE(gate.tryAcquire(ControlOperationKind::NONE, reservation));
    TEST_ASSERT_FALSE(reservation.valid());
    TEST_ASSERT_FALSE(gate.isHeld());
}

TEST_CASE("control operation gate exposes a stable operation id and final result",
          "[control][gate]")
{
    ControlOperationGate gate;
    ControlOperationReservation reservation;
    TEST_ASSERT_TRUE(gate.tryAcquire(ControlOperationKind::CONFIGURATION,
                                     reservation,
                                     42));

    const auto current = gate.currentStatus();
    TEST_ASSERT_TRUE(current.known);
    TEST_ASSERT_TRUE(current.active);
    TEST_ASSERT_EQUAL_UINT64(42, current.operationId);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlOperationPhase::RESERVED),
                          static_cast<int>(current.phase));

    TEST_ASSERT_TRUE(gate.updatePhase(
        reservation, ControlOperationPhase::WRITING, -7));
    TEST_ASSERT_TRUE(gate.release(
        reservation, ControlOperationPhase::FAILED, -7));

    const auto last = gate.lastStatus();
    TEST_ASSERT_TRUE(last.known);
    TEST_ASSERT_FALSE(last.active);
    TEST_ASSERT_EQUAL_UINT64(42, last.operationId);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ControlOperationPhase::FAILED),
                          static_cast<int>(last.phase));
    TEST_ASSERT_EQUAL_INT(-7, last.resultCode);
}

TEST_CASE("control operation gate blocks non-configuration work during recovery",
          "[control][gate][recovery]")
{
    ControlOperationGate gate;
    ControlOperationReservation motion;
    ControlOperationReservation configuration;

    gate.setRecoveryPending(true);
    TEST_ASSERT_TRUE(gate.recoveryPending());
    TEST_ASSERT_FALSE(gate.tryAcquire(ControlOperationKind::MOTION, motion));
    TEST_ASSERT_FALSE(gate.tryAcquire(ControlOperationKind::OTA, motion));
    TEST_ASSERT_TRUE(gate.tryAcquire(ControlOperationKind::CONFIGURATION,
                                     configuration, 123));
    TEST_ASSERT_TRUE(gate.release(configuration,
                                  ControlOperationPhase::ABORTED,
                                  ESP_ERR_INVALID_STATE));

    gate.setRecoveryPending(false);
    TEST_ASSERT_TRUE(gate.tryAcquire(ControlOperationKind::MOTION, motion));
    TEST_ASSERT_TRUE(gate.release(motion));
}

TEST_CASE("control operation gate permits OTA repair while bundle recovery is pending",
          "[control][gate][ota][recovery]")
{
    ControlOperationGate gate;
    ControlOperationReservation ota;
    ControlOperationReservation motion;
    ControlOperationReservation configuration;

    gate.setOtaRecoveryPending(true);
    TEST_ASSERT_TRUE(gate.recoveryPending());
    TEST_ASSERT_TRUE(gate.otaRecoveryPending());
    TEST_ASSERT_FALSE(gate.tryAcquire(ControlOperationKind::MOTION, motion));
    TEST_ASSERT_FALSE(gate.tryAcquire(ControlOperationKind::CALIBRATION, motion));
    TEST_ASSERT_TRUE(gate.tryAcquire(ControlOperationKind::OTA, ota));
    TEST_ASSERT_TRUE(gate.release(ota, ControlOperationPhase::ABORTED,
                                  ESP_ERR_INVALID_STATE));

    TEST_ASSERT_TRUE(gate.tryAcquire(ControlOperationKind::CONFIGURATION,
                                     configuration));
    TEST_ASSERT_TRUE(gate.release(configuration));

    gate.setOtaRecoveryPending(false);
    TEST_ASSERT_FALSE(gate.recoveryPending());
    TEST_ASSERT_TRUE(gate.tryAcquire(ControlOperationKind::MOTION, motion));
    TEST_ASSERT_TRUE(gate.release(motion));
}
