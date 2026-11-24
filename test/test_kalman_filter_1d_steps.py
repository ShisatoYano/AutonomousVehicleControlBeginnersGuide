"""test_kalman_filter_1d_steps.py

Author: Zeyun DU
"""

import sys
from pathlib import Path

import pytest

sys.path.append(
    str(Path(__file__).resolve().parents[1] / "src/simulations/localization/kalman_filter_1d")
)
from kalman_filter_1d_pos import KalmanFilter1dPos


def test_prediction_advances_position_and_adds_velocity_noise_to_variance() -> None:
    kf = KalmanFilter1dPos(
        interval_sec=0.5, input_velocity_ms=4, input_noise_variance=8, observation_noise_variance=3
    )

    # In half a second, 4 m/s adds 2 m and velocity noise adds 0.5² × 8 = 2 m².
    assert kf.predict_position(x_prev=1, u=4) == pytest.approx(3)
    assert kf.predict_variance(p_prev=1) == pytest.approx(3)


@pytest.mark.parametrize("noise, error_variance, gain", [(1, 100, 0.99), (9801, 9900, 0.01)])
def test_gain_decreases_as_observation_noise_increases(
    noise: float, error_variance: float, gain: float
) -> None:
    kf = KalmanFilter1dPos(
        interval_sec=1,
        input_velocity_ms=0,
        input_noise_variance=0,
        observation_noise_variance=noise,
    )

    variance = kf.calculate_obsrv_pred_err_var(p_pred=99)
    assert variance == pytest.approx(error_variance)
    assert kf.calculate_kalman_gain(99, variance) == pytest.approx(gain)


@pytest.mark.parametrize("observation, expected_position", [(7, 5), (-3, 0)])
def test_estimate_moves_halfway_to_observation_and_halves_predicted_variance(
    observation: float,
    expected_position: float,
) -> None:
    kf = KalmanFilter1dPos(
        interval_sec=0.5, input_velocity_ms=4, input_noise_variance=8, observation_noise_variance=3
    )

    # Predicted position and variance are both 3. Equal variances give gain 1/2.
    position, variance = kf.estimate_position(x_prev=1, p_prev=1, u=4, z=observation)
    assert position == pytest.approx(expected_position)
    assert variance == pytest.approx(1.5)
