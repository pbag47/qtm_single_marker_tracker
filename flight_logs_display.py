import csv
import numpy as np
import plotly.graph_objects as go
from dash import dcc, html, Dash


def extract_csv_data(filename):
    file = open(filename, 'r')
    reader = csv.DictReader(file)
    data_list = list(reader)
    bodies_names = []
    variables_names = ['QTM packet timestamp (s)']
    sampling_frequency = 25  # Hz
    for key in data_list[0].keys():
        if key != 'QTM packet timestamp (s)':
            k = key.split()
            name = k[0]
            variable = k[1]
            try:
                _ = bodies_names.index(name)
            except ValueError:
                bodies_names.append(name)
            try:
                _ = variables_names.index(variable)
            except ValueError:
                variables_names.append(variable)
        else:
            sampling_period = float(data_list[1][key]) - float(data_list[0][key])
            sampling_frequency = round(1 / sampling_period)
    data = {body_name: {variable_name: [] for variable_name in variables_names} for body_name in bodies_names}
    for key in data_list[0].keys():
        if key == 'QTM packet timestamp (s)':
            for body_name in bodies_names:
                for i in range(len(data_list)):
                    data[body_name][key].append(data_list[i][key])
        else:
            k = key.split()
            name = k[0]
            variable = k[1]
            for i in range(len(data_list)):
                data[name][variable].append(data_list[i][key])
    return data, len(data_list), sampling_frequency


def plot_3d_trajectory(figure, flight_zone_boundaries, colors_association, sampling_frequency, samples_number):
    global LOGS_DATA

    # Plot x, y, and z vectors at the origin
    figure.add_trace(go.Scatter3d(x=[0, 1], y=[0, 0], z=[0, 0], line=dict(color='red', width=20),
                                  name='X vector', legendgroup='0', mode='lines'))
    figure.add_trace(go.Scatter3d(x=[0, 0], y=[0, 1], z=[0, 0], line=dict(color='green', width=20),
                                  name='Y vector', legendgroup='0', mode='lines'))
    figure.add_trace(go.Scatter3d(x=[0, 0], y=[0, 0], z=[0, 1], line=dict(color='blue', width=20),
                                  name='Z vector', legendgroup='0', mode='lines'))

    # Plot cf trajectories
    for cf_name in LOGS_DATA.keys():
        color = colors_association[cf_name]
        figure.add_trace(go.Scatter3d(x=[LOGS_DATA[cf_name]['x'][0]],
                                      y=[LOGS_DATA[cf_name]['y'][0]],
                                      z=[LOGS_DATA[cf_name]['z'][0]],
                                      line=dict(color=color),
                                      name=cf_name,
                                      legendgroup=cf_name,
                                      mode='markers',
                                      marker=dict(size=4)
                                      ))

    # Create frames for 3D graph animation
    frame_dicts = []
    indices = np.round(np.linspace(0, samples_number - 1, round(samples_number / (0.5 * sampling_frequency))))
    for i in indices:
        vector_frames_data_list = [go.Scatter3d(x=[0, 1], y=[0, 0], z=[0, 0]),
                                   go.Scatter3d(x=[0, 0], y=[0, 1], z=[0, 0]),
                                   go.Scatter3d(x=[0, 0], y=[0, 0], z=[0, 1])]
        cf_frame_data_list = [go.Scatter3d(x=[LOGS_DATA[cf_name]['x'][int(i)]],
                                           y=[LOGS_DATA[cf_name]['y'][int(i)]],
                                           z=[LOGS_DATA[cf_name]['z'][int(i)]]
                                           ) for cf_name in LOGS_DATA.keys()]
        data = vector_frames_data_list + cf_frame_data_list
        frame_dict = dict(data=data, name=str(int(i)))
        frame_dicts.append(frame_dict)
    figure.frames = frame_dicts
    figure.update_scenes(xaxis=dict(range=[flight_zone_boundaries[0][0], flight_zone_boundaries[0][1]],
                                    title='X (m)'),
                         yaxis=dict(range=[flight_zone_boundaries[1][0], flight_zone_boundaries[1][1]],
                                    title='Y (m)'),
                         zaxis=dict(range=[flight_zone_boundaries[2][0], flight_zone_boundaries[2][1]],
                                    title='Z (m)'),
                         aspectmode='manual',
                         aspectratio=dict(x=1,
                                          y=abs((flight_zone_boundaries[1][1] - flight_zone_boundaries[1][0]) /
                                                (flight_zone_boundaries[0][1] - flight_zone_boundaries[0][0])),
                                          z=abs((flight_zone_boundaries[2][1] - flight_zone_boundaries[2][0]) /
                                                (flight_zone_boundaries[0][1] - flight_zone_boundaries[0][0]))
                                          ))
    return figure


if __name__ == '__main__':
    flight_area_limits = [[-2.5, 2.5], [-2.5, 2.5], [-0.1, 2]]
    frame_setup = dict(duration=500, redraw=True)
    transition_setup = dict(duration=100, easing='linear')
    cf_colors = dict(cf1='black',
                     cf2='cyan',
                     cf3='yellow',
                     cf4='magenta',
                     cf5='blue',
                     cf6='red',
                     cf7='green',
                     cf8='pink',
                     cf9='brown',
                     cf10='orange')

    LOGS_DATA, step_number, sample_rate = extract_csv_data('logs.csv')

    fig = go.Figure()
    fig = plot_3d_trajectory(fig, flight_area_limits, cf_colors, sample_rate, step_number)

    app = Dash()
    app.layout = html.Div([dcc.Graph(id='3d_graph', figure=fig, animate=True)])

    slider = dict(pad=dict(b=10,
                           t=60),
                  len=0.9,
                  x=0.1,
                  y=0,
                  steps=[dict(args=[[f.name], dict(frame=frame_setup,
                                                   mode='immediate',
                                                   fromcurrent=True,
                                                   transition=transition_setup)],
                              label=str(k),
                              method='animate') for k, f in enumerate(fig.frames)])

    fig.update_layout(title_text='Flight logs',
                      # template='plotly_dark',
                      legend_tracegroupgap=10,
                      updatemenus=[dict(type='buttons',
                                        buttons=[dict(label='Play',
                                                      method='animate',
                                                      args=[None, dict(frame=frame_setup)]),
                                                 dict(label='Pause',
                                                      method='animate',
                                                      args=[[None], dict(frame=dict(duration=0,
                                                                                    redraw=True))
                                                            ])],
                                        direction='left',
                                        pad=dict(r=10, t=70),
                                        x=0.1,
                                        y=0
                                        )
                                   ],
                      sliders=[slider]
                      )

    app.run_server()
