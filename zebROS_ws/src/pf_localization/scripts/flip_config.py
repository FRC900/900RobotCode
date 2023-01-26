import yaml
config_root = "/home/ubuntu/2023RobotCode/zebROS_ws/src/pf_localization/config"

with open(config_root + "/2022Rumble_params.yaml") as file_:
    data = yaml.load(file_, Loader=yaml.Loader)
print(data)
print("\n")

# Define where the new 0,0 should be
new_x = 8.229
new_y = 4.114
beacons = data["beacons"]
for beacon in beacons: 
    beacon[0] = round(beacon[0] - new_x, 3)
    beacon[1] = round(beacon[1] - new_y, 3)
    

data["beacons"] = beacons
print(data)
with open(config_root + "/2022Rumble_params_center.yaml", 'w') as file:
    documents = yaml.dump(data, file)
