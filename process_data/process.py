num_pose = 13741

position_list = []
id_list = [] 
orientation_list = []
pose_time_list = []
has_covariance_list = []
sensor_measurement_list = []


current_sensor_measurement = []
sensor_measurement_size = 0


import os
# we should cd in process_data

filePath = "/home/ubuntu/multi-cue-fusion-ws/data/tss_datasetA_full.yaml"

if os.path.exists(filePath):
    print("file exists")
    os.remove(filePath)
else:
    print("file non-exists")


with open("/home/ubuntu/multi-cue-fusion-ws/process_data/old_tss_datasetA_full.yaml", "r") as f:

    lines = f.readlines()
    for index, line in enumerate(lines):
        if  "position" in line:
            position_list.append(lines[index:index + 4])

        elif "id" in line:
            id_list.append(line)
        elif "orienta" in line:
            orientation_list.append(lines[index:index + 5])
            pass
        elif "pose_time" in line:
            pose_time_list.append(line)
            pass
        elif "covar" in line:
            has_covariance_list.append(line)
            pass
        elif "or_measureme" in line:
            sensor_measurement_list.append(line)
            pass
        elif line == "    -\n": 
            sensor_measurement_list.append(lines[index:index + 4])
            pass
        else:
            continue

chunk = []
# process sensor_measurement_list
new_sensor_measurement_list = []
for item in sensor_measurement_list:
    if "sensor_measurements:" in item:
        new_sensor_measurement_list.append(chunk) # save the last chunk
        chunk = []
        chunk.append(item)
    else:
        chunk += item
        pass


new_sensor_measurement_list = new_sensor_measurement_list[1:]
new_sensor_measurement_list.append(sensor_measurement_list[-1])

[print(len(this_list)) for this_list in [position_list, id_list, orientation_list, pose_time_list, \
        has_covariance_list, new_sensor_measurement_list]]

with open("/home/ubuntu/multi-cue-fusion-ws/data/tss_datasetA_full.yaml","a+") as my_file:
    for items_6 in zip(position_list, id_list, orientation_list, pose_time_list, \
        has_covariance_list, new_sensor_measurement_list):
        
        for i_list in items_6:
            for j in range(len(i_list)):
                my_file.write(i_list[j])
            
            pass
