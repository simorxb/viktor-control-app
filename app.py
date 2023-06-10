import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import control as ct
import math

from viktor import ViktorController
from viktor.parametrization import ViktorParametrization, NumberField, TextField, LineBreak, OutputField, Text
from viktor.views import PlotlyView, PlotlyResult


def calc_gain_margin(params, **kwargs):
    (gm_dB, pm, dm) = calc_margins(params)
    return round(gm_dB, 2)

def calc_phase_margin(params, **kwargs):
    (gm_dB, pm, dm) = calc_margins(params)
    return round(pm, 2)

def calc_delay_margin(params, **kwargs):
    (gm_dB, pm, dm) = calc_margins(params)
    return round(dm, 2)

def calc_margins(params):

    s = ct.TransferFunction.s

    # System transfer function
    P = eval(params.tf)

    # PID parameters
    kp = params.kp
    ki = params.ki
    kd = params.kd
    tau = params.tau
    
    # PID transfer function
    C = kp + ki/s + kd*s/(tau*s+1)

    # Calculate the gain and phase margins
    gm, pm, gc, pc = ct.margin(C*P)

    # Gain margin in dB
    gm_dB = 20*np.log10(gm)

    # Inf not compatible with output field
    if math.isinf(gm_dB):
        gm_dB = 10000

    # Delay margin
    dm = ((pm*math.pi/180)/pc)

    return (gm_dB, pm, dm)


# Parametrization class - contains input fields.
class Parametrization(ViktorParametrization):
    welcome = Text("""## Welcome to Simone's Control Application!

The application uses the Python control library to allow you to define your plant transfer function, tune the PID, see the system response, gain margin, phase margin and delay margin of the close loop system. The PID transfer function is assumed to be:

$$
C(s) = k_p + \\frac{k_i}{s} + \\frac{k_d s}{\\tau s+1}
$$

If you are interested in control engineering follow me on Linkedin [Simone Bertoni](https://www.linkedin.com/in/simone-bertoni-control-eng) or have a look at my course [PID Control - Things I Wish I Knew When I graduated](https://simonebertoni.thinkific.com).""")
    
    info1 = Text("### PID Configuration")
    kp = NumberField("Proportional gain - $k_p$", default=0.5, step=0.1, min=0)
    ki = NumberField("Integral gain - $k_i$", default=0, step=0.1, min=0)
    kd = NumberField("Derivative gain - $k_d$", default=0, step=0.1, min=0)
    tau = NumberField("Derivative time constant - $\\tau$", default=1, step=0.1, suffix="s", min=0)
    new_line1 = LineBreak()
    info2 = Text("### Simulation Configuration")
    end_time = NumberField("End time", default=20, step=0.1, suffix="s", min=0.1)
    new_line2 = LineBreak()
    info3 = Text("### Plant Configuration")
    tf = TextField('Plant transfer function', default='1/((s+0.1)*(s+0.3)*(s+3))')
    new_line5 = LineBreak()
    info4 = Text("### Margins")
    gm = OutputField('Gain margin', value=calc_gain_margin, suffix='dB')
    pm = OutputField('Phase margin', value=calc_phase_margin, suffix='Deg')
    dm = OutputField('Delay margin', value=calc_delay_margin, suffix='s')


# Controller class - to show stuff in the app
class Controller(ViktorController):
    label = "Control App"
    parametrization = Parametrization

    @PlotlyView("Tune Step Response", duration_guess=1)
    def step_response_plot(self, params, **kwargs):
        
        s = ct.TransferFunction.s

        # System transfer function
        P = eval(params.tf)

        # PID parameters
        kp = params.kp
        ki = params.ki
        kd = params.kd
        tau = params.tau

        # End time
        end_time = params.end_time

        # PID transfer function
        C = kp + ki/s + kd*s/(tau*s+1)

        # Close-loop transfer function
        G_P = ct.feedback(C*P, 1)

        # Transfer function from setpoint to controller output
        G_P_u = C/(1+C*P)

        # Step response
        t, y = ct.step_response(G_P, T=end_time)

        # Step response - controller output
        t_u, y_u = ct.step_response(G_P_u, T=end_time)

        # Create a subplot with Plotly
        fig = make_subplots(rows=2, cols=1)

        # Add plots to the subplot grid
        fig.add_trace(go.Scatter(x=t, y=y, name="System Response"), row=1, col=1)
        fig.add_trace(go.Scatter(x=t_u, y=y_u, name="Controller Output"), row=2, col=1)
        fig.update_xaxes(title_text="Time (s)", row=1, col=1)
        fig.update_yaxes(title_text="System Step Response", row=1, col=1)
        fig.update_xaxes(title_text="Time (s)", row=2, col=1)
        fig.update_yaxes(title_text="Controller Output", row=2, col=1)

        return PlotlyResult(fig.to_json())
    