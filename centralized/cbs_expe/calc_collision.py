
import os
import yaml
import argparse
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_with", help="output file with the schedule with ours")
    parser.add_argument("output_without", help="output file with the schedule without ours")
    parser.add_argument("dynamic_obs", help="path of dynamic obs")
    parser.add_argument("result_pth", help="path of results")
    
    args = parser.parse_args()
    output_with_pth = args.output_with
    output_without_pth = args.output_without
    dy_obs_pth = args.dynamic_obs
    result_pth = args.result_pth

    with open(output_with_pth, 'r') as file:
        try:
            output_with_file = yaml.load(file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    with open(output_without_pth, 'r') as file:
        try:
            output_without_file = yaml.load(file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    with open(dy_obs_pth, 'r') as file:
        try:
            dy_obs_file = yaml.load(file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    a_max_t_with = max([len(path) for path in output_with_file['schedule'].values()])
    a_max_t_without = max([len(path) for path in output_without_file['schedule'].values()])
    d_max_t = max([len(path) for path in dy_obs_file['schedule'].values()])

    max_t_with= max(a_max_t_with, d_max_t)
    max_t_without= max(a_max_t_without, d_max_t)

    edge_conflict_num_with = 0
    edge_conflict_num_without = 0

    for t in range(max_t_with):
        for agent_name in output_with_file['schedule'].keys():
            for dy_obs_name in dy_obs_file['schedule'].keys():
                a_pth = output_with_file['schedule'][agent_name]
                d_pth = dy_obs_file['schedule'][dy_obs_name]
                
                if t >= len(a_pth) - 1 or t >= len(d_pth) - 1:
                    continue

                state_1a = a_pth[t]
                state_2a = a_pth[t + 1]
                state_1d = d_pth[t]
                state_2d = d_pth[t + 1]

                if ((state_1a['x'] == state_2d['x']) and (state_1a['y'] == state_2d['y'])) and ((state_2a['x'] == state_1d['x']) and (state_2a['y'] == state_1d['y'])):
                    edge_conflict_num_with += 1

    for t in range(max_t_without):
        for agent_name in output_without_file['schedule'].keys():
            for dy_obs_name in dy_obs_file['schedule'].keys():
                a_pth = output_without_file['schedule'][agent_name]
                d_pth = dy_obs_file['schedule'][dy_obs_name]
                
                if t >= len(a_pth) - 1 or t >= len(d_pth) - 1:
                    continue

                state_1a = a_pth[t]
                state_2a = a_pth[t + 1]
                state_1d = d_pth[t]
                state_2d = d_pth[t + 1]

                if ((state_1a['x'] == state_2d['x']) and (state_1a['y'] == state_2d['y'])) and ((state_2a['x'] == state_1d['x']) and (state_2a['y'] == state_1d['y'])):
                    edge_conflict_num_without += 1


    output_with_points = []
    for agent in output_with_file['schedule'].keys():
        for point in output_with_file['schedule'][agent]:
            output_with_points.append(point)

    output_without_points = []
    for agent in output_without_file['schedule'].keys():
        for point in output_without_file['schedule'][agent]:
            output_without_points.append(point)

    dy_obs_points = []
    for agent in dy_obs_file['schedule'].keys():
        for point in dy_obs_file['schedule'][agent]:
            dy_obs_points.append(point)

    collision_num_with = 0
    for out_pt in output_with_points:
        for dy_pt in dy_obs_points:
            if out_pt == dy_pt:
                collision_num_with += 1
    
    collision_num_without = 0
    for out_pt in output_without_points:
        for dy_pt in dy_obs_points:
            if out_pt == dy_pt:
                collision_num_without += 1
    
    f = open(result_pth, 'a')
    f.write('Collision with: ' + str(collision_num_with + edge_conflict_num_with) + '  |  Collision without: ' + str(collision_num_without + edge_conflict_num_without) + ' \n')
    f.close()
    
if __name__=='__main__':
    main()