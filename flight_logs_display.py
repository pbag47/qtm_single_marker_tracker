import csv
import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots


def extract_csv_data(filename):
    file = open(filename, 'r')
    reader = csv.DictReader(file)
    data_list = list(reader)
    body_names = set(data_list_element['Crazyflie name'] for data_list_element in data_list)
    data = {body_name: {key: [] for key in data_list[0].keys() if key != 'Crazyflie name'}
            for body_name in body_names}
    for i in range(len(data_list)):
        for key in data_list[0].keys():
            if key != 'Crazyflie name':
                data[data_list[i]['Crazyflie name']][key].append(float(data_list[i][key]))
    return data


def plot_3d_trajectory(trajectory, flight_zone_boundaries, colors_association, fig):
    # Plot x, y, and z vectors at the origin
    fig.add_trace(go.Scatter3d(x=[0, 1], y=[0, 0], z=[0, 0], line=dict(color='red', width=20),
                               name='X vector', legendgroup='0', mode='lines'),
                  row=1, col=1)
    fig.add_trace(go.Scatter3d(x=[0, 0], y=[0, 1], z=[0, 0], line=dict(color='green', width=20),
                               name='Y vector', legendgroup='0', mode='lines'),
                  row=1, col=1)
    fig.add_trace(go.Scatter3d(x=[0, 0], y=[0, 0], z=[0, 1], line=dict(color='blue', width=20),
                               name='Z vector', legendgroup='0', mode='lines'),
                  row=1, col=1)

    # Plot cf trajectories
    i = 0
    for cf_name in trajectory.keys():
        color = colors_association[cf_name]
        i = i + 1
        # Main line
        fig.add_trace(go.Scatter3d(x=trajectory[cf_name]['QTM_x (m)'],
                                   y=trajectory[cf_name]['QTM_y (m)'],
                                   z=trajectory[cf_name]['QTM_z (m)'],
                                   line=dict(color=color),
                                   name=cf_name + ' actual state',
                                   legendgroup=str(i),
                                   mode='lines'
                                   ),
                      row=1, col=1)

        # Start point
        fig.add_trace(go.Scatter3d(x=[trajectory[cf_name]['QTM_x (m)'][0]],
                                   y=[trajectory[cf_name]['QTM_y (m)'][0]],
                                   z=[trajectory[cf_name]['QTM_z (m)'][0]],
                                   line=dict(color=color),
                                   legendgroup=str(i),
                                   showlegend=False,
                                   mode='markers',
                                   marker=dict(size=4)
                                   ),
                      row=1, col=1)

        # Stop point
        fig.add_trace(go.Scatter3d(x=[trajectory[cf_name]['QTM_x (m)'][-1]],
                                   y=[trajectory[cf_name]['QTM_y (m)'][-1]],
                                   z=[trajectory[cf_name]['QTM_z (m)'][-1]],
                                   line=dict(color=color),
                                   legendgroup=str(i),
                                   showlegend=False,
                                   mode='markers',
                                   marker=dict(symbol='cross', size=4)
                                   ),
                      row=1, col=1)

    fig.update_scenes(xaxis=dict(range=[flight_zone_boundaries[0][0], flight_zone_boundaries[0][1]],
                                 title='X (m)'),
                      yaxis=dict(range=[flight_zone_boundaries[1][0], flight_zone_boundaries[1][1]],
                                 title='Y (m)'),
                      zaxis=dict(range=[flight_zone_boundaries[2][0], flight_zone_boundaries[2][1]],
                                 title='Z (m)'),
                      aspectmode='manual',
                      aspectratio=dict(x=1,
                                       y=abs((flight_zone_boundaries[1][1] - flight_zone_boundaries[1][0]) /
                                             (flight_zone_boundaries[0][1] - flight_zone_boundaries[0][0])
                                             ),
                                       z=abs((flight_zone_boundaries[2][1] - flight_zone_boundaries[2][0]) /
                                             (flight_zone_boundaries[0][1] - flight_zone_boundaries[0][0])
                                             )
                                       ),
                      row=1, col=1)
    return fig


def plot_attitude_vs_time(data, colors_association, fig):
    i = 0
    for cf_name in data.keys():
        i = i + 1
        color = colors_association[cf_name]

        qtm_xn = []
        qtm_yn = []
        xn_g = []
        yn_g = []
        for j in range(len(data[cf_name]['QTM packet timestamp (s)'])):
            qtm_xn.append(data[cf_name]['QTM_x (m)'][j] * np.cos(data[cf_name]['cf_yaw (°)'][j] * np.pi / 180)
                          + data[cf_name]['QTM_y (m)'][j] * np.sin(data[cf_name]['cf_yaw (°)'][j] * np.pi / 180))

            qtm_yn.append(- data[cf_name]['QTM_x (m)'][j] * np.sin(data[cf_name]['cf_yaw (°)'][j] * np.pi / 180)
                          + data[cf_name]['QTM_y (m)'][j] * np.cos(data[cf_name]['cf_yaw (°)'][j] * np.pi / 180))

            xn_g.append(data[cf_name]['x_g (m)'][j] * np.cos(data[cf_name]['cf_yaw (°)'][j] * np.pi / 180)
                          + data[cf_name]['y_g (m)'][j] * np.sin(data[cf_name]['cf_yaw (°)'][j] * np.pi / 180))

            yn_g.append(- data[cf_name]['x_g (m)'][j] * np.sin(data[cf_name]['cf_yaw (°)'][j] * np.pi / 180)
                          + data[cf_name]['y_g (m)'][j] * np.cos(data[cf_name]['cf_yaw (°)'][j] * np.pi / 180))

        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['z_g (m)'],
                                 line=dict(color=color, dash='dash'),
                                 name=cf_name + ' targeted state',
                                 legendgroup=str(i),
                                 mode='lines',
                                 ),
                      row=1, col=4)
        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['QTM_z (m)'],
                                 line=dict(color=color),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=1, col=4)
        fig.update_xaxes(title='Time (s)', row=1, col=4)
        fig.update_yaxes(title='Z (m)', row=1, col=4)

        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['QTM_vz (m/s)'],
                                 line=dict(color=color),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=2, col=4)
        fig.update_xaxes(title='Time (s)', row=2, col=4)
        fig.update_yaxes(title='VZ (m/s)', row=2, col=4)

        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['PID_zp'],
                                 line=dict(color=color),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=3, col=4)
        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['PID_zi'],
                                 line=dict(color=color, dash='dot'),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=3, col=4)
        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['PID_zd'],
                                 line=dict(color=color, dash='dash'),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=3, col=4)
        fig.update_xaxes(title='Time (s)', row=3, col=4)
        fig.update_yaxes(title='Thrust (PWM)', row=3, col=4)

        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['thrust_c (PWM)'],
                                 line=dict(color=color),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=4, col=4)
        fig.update_xaxes(title='Time (s)', row=4, col=4)
        fig.update_yaxes(title='Thrust command (PWM)', row=4, col=4, range=[0, 66000])

        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=qtm_xn,
                                 line=dict(color=color),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=3, col=1)
        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=xn_g,
                                 line=dict(color=color, dash='dash'),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=3, col=1)
        fig.update_xaxes(title='Time (s)', row=3, col=1)
        fig.update_yaxes(title='Xn (m)', row=3, col=1)

        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['roll_c (°)'],
                                 line=dict(color=color),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=4, col=1)
        fig.update_xaxes(title='Time (s)', row=4, col=1)
        fig.update_yaxes(title='Roll command (°)', row=4, col=1)

        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=qtm_yn,
                                 line=dict(color=color),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=3, col=2)
        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=yn_g,
                                 line=dict(color=color, dash='dash'),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=3, col=2)
        fig.update_xaxes(title='Time (s)', row=3, col=2)
        fig.update_yaxes(title='Yn (m)', row=3, col=2)

        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['pitch_c (°)'],
                                 line=dict(color=color),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines',
                                 ),
                      row=4, col=2)
        fig.update_xaxes(title='Time (s)', row=4, col=2)
        fig.update_yaxes(title='Pitch command (°)', row=4, col=2)

        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['cf_yaw (°)'],
                                 line=dict(color=color),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines'
                                 ),
                      row=3, col=3)
        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['yaw_g (°)'],
                                 line=dict(color=color, dash='dash'),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines'
                                 ),
                      row=3, col=3)
        fig.update_xaxes(title='Time (s)', row=3, col=3)
        fig.update_yaxes(title='Yaw (°)', row=3, col=3)

        fig.add_trace(go.Scatter(x=data[cf_name]['QTM packet timestamp (s)'],
                                 y=data[cf_name]['yaw_c (°)'],
                                 line=dict(color=color),
                                 legendgroup=str(i),
                                 showlegend=False,
                                 mode='lines'
                                 ),
                      row=4, col=3)
        fig.update_xaxes(title='Time (s)', row=4, col=3)
        fig.update_yaxes(title='Yaw (°)', row=4, col=3)
    return fig


if __name__ == '__main__':
    flight_area_limits = [[-2.5, 2.5], [-2.5, 2.5], [0, 2]]
    cf_colors = {'cf1': 'white',
                 'cf2': 'cyan',
                 'cf3': 'yellow',
                 'cf4': 'magenta'}

    logs_data = extract_csv_data('logs.csv')
    figure = make_subplots(rows=4, cols=4,
                           specs=[[{'type': 'scatter3d', 'rowspan': 2, 'colspan': 3}, None, None, {'type': 'scatter'}],
                                  [None, None, None, {'type': 'scatter'}],
                                  [{'type': 'scatter'}, {'type': 'scatter'}, {'type': 'scatter'}, {'type': 'scatter'}],
                                  [{'type': 'scatter'}, {'type': 'scatter'}, {'type': 'scatter'}, {'type': 'scatter'}]],
                           subplot_titles=('3D trajectory', 'Target Z & actual Z vs time',
                                           'VZ vs time', 'Target Xn & Xn vs time', 'Target Yn & Yn vs time',
                                           'Target yaw & Measured yaw vs time', 'Z PID components vs time',
                                           'Roll command vs time', 'Pitch command vs time', 'Yaw command vs time',
                                           'Thrust command vs time'),
                           shared_xaxes=True,
                           horizontal_spacing=0.05)
    figure.update_layout(title_text='Flight logs', template='plotly_dark', legend_tracegroupgap=50)

    figure = plot_3d_trajectory(logs_data, flight_area_limits, cf_colors, figure)
    figure = plot_attitude_vs_time(logs_data, cf_colors, figure)

    figure.show()
