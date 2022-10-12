import numpy as np
import matplotlib.pyplot as plt

from extremum_seeking_control import esc


def cost_function(speed, sideslip):
    return 100 + speed**2 + 50 * np.power(sideslip - np.pi / 2, 2)


def run_simulation():

    # Simulation Setup
    dt = 0.02
    end_time = 500.0

    init_speed = 5.0  # in m/s
    init_sideslip = 40 * np.pi / 180  # in deg

    speed_disturbance_magnitude = 0.5  # [m/s]
    speed_frequency = 1.0  # [rad/s]
    sideslip_disturbance_magnitude = 7.5 * np.pi / 180
    sideslip_frequency = 0.5  # [rad/s]
    integrator_gain = -0.05
    step_num = np.int(end_time / dt)

    # Construct controller
    speed_extremum_seeking = esc(
        dt,
        speed_frequency,
        speed_disturbance_magnitude,
        integrator_gain,
        init_speed,
        speed_frequency,
        speed_frequency,
        True,
    )
    sideslip_extremum_seeking = esc(
        dt,
        sideslip_frequency,
        sideslip_disturbance_magnitude,
        integrator_gain,
        init_sideslip,
        sideslip_frequency,
        sideslip_frequency,
        True,
    )

    # Logging data
    time_log = np.zeros((step_num, 1))
    cost_log = np.zeros((step_num, 1))

    speed_es_output_log = np.zeros((step_num, 1))
    speed_high_pass_output_log = np.zeros((step_num, 1))
    speed_low_pass_output_log = np.zeros((step_num, 1))
    speed_m2_sqrt_log = np.zeros((step_num, 1))
    speed_gradient_log = np.zeros((step_num, 1))
    speed_setpoint_log = np.zeros((step_num, 1))

    sideslip_es_output_log = np.zeros((step_num, 1))
    sideslip_high_pass_output_log = np.zeros((step_num, 1))
    sideslip_low_pass_output_log = np.zeros((step_num, 1))
    sideslip_m2_sqrt_log = np.zeros((step_num, 1))
    sideslip_gradient_log = np.zeros((step_num, 1))
    sideslip_setpoint_log = np.zeros((step_num, 1))

    t = 0
    current_speed = init_speed
    current_sideslip = init_sideslip
    cost_init = cost_function(init_speed, init_sideslip)
    for i in range(step_num):
        cost = cost_function(current_speed, current_sideslip)
        [
            speed_es_output,
            speed_high_pass_output,
            speed_low_pass_output,
            speed_m2_sqrt,
            speed_gradient,
            speed_setpoint,
        ] = speed_extremum_seeking.update(cost - cost_init)
        [
            sideslip_es_output,
            sideslip_high_pass_output,
            sideslip_low_pass_output,
            sideslip_m2_sqrt,
            sideslip_gradient,
            sideslip_setpoint,
        ] = sideslip_extremum_seeking.update(cost - cost_init)

        # log data
        time_log[i] = t
        cost_log[i] = cost
        speed_es_output_log[i] = current_speed
        speed_high_pass_output_log[i] = speed_high_pass_output
        speed_low_pass_output_log[i] = speed_low_pass_output
        speed_m2_sqrt_log[i] = speed_m2_sqrt
        speed_gradient_log[i] = speed_gradient
        speed_setpoint_log[i] = speed_setpoint
        sideslip_es_output_log[i] = current_sideslip
        sideslip_high_pass_output_log[i] = sideslip_high_pass_output
        sideslip_low_pass_output_log[i] = sideslip_low_pass_output
        sideslip_m2_sqrt_log[i] = sideslip_m2_sqrt
        sideslip_gradient_log[i] = sideslip_gradient
        sideslip_setpoint_log[i] = sideslip_setpoint

        t += dt
        current_speed = speed_es_output
        current_sideslip = sideslip_es_output

    # Generate plots

    fig, ax = plt.subplots(6, 1, sharex=True)

    ax[0].plot(time_log, speed_setpoint_log, "b--", label="setpoint")
    ax[0].plot(time_log, speed_es_output_log, "b", label="output")
    ax[0].set_ylabel("speed [m/s]")
    ax[0].legend()

    ax[1].plot(time_log, sideslip_setpoint_log * 180 / np.pi, "r--", label="setpoint")
    ax[1].plot(time_log, sideslip_es_output_log * 180 / np.pi, "r", label="output")
    ax[1].set_ylabel("sideslip [deg]")
    ax[1].legend()

    ax[2].plot(time_log, cost_log)
    ax[2].set_ylabel("cost")

    ax[3].plot(time_log, speed_gradient_log, "b", label="speed gradient")
    ax[3].plot(time_log, speed_m2_sqrt_log, "b--", label="speed sqrt m2")
    ax[3].plot(time_log, sideslip_gradient_log, "r", label="ssg radient")
    ax[3].plot(time_log, sideslip_m2_sqrt_log, "r--", label="ss sqrt m2")
    ax[3].legend()

    ax[4].plot(time_log, speed_high_pass_output_log, "b", label="speed hp out")
    ax[4].plot(time_log, speed_low_pass_output_log, "b--", label="speed lp out")
    ax[4].legend()

    ax[5].plot(time_log, sideslip_high_pass_output_log, "r", label="ss hp out")
    ax[5].plot(time_log, sideslip_low_pass_output_log, "r--", label="ss lp out")
    ax[5].legend()

    ax[-1].set_xlabel("time [s]")
    plt.show()


if __name__ == "__main__":
    run_simulation()
