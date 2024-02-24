from scipy.spatial.transform import Rotation
import yaml

data = dict(
    tags = [
        dict(
            id = 1,
            size = 0.165
        ),
        dict(
            id = 2,
            size = 0.165
        )
    ]
)

with open('data.yml', 'w') as outfile:
    yaml.dump(data, outfile, default_flow_style=False)
'''     
tags:
- id: 7
    size: 0.16510000
    pose:
    position:
        x: -0.01270000
        y: 5.54786800
        z: 1.45110200
    rotation:
        x: 1.20919970
        y: 1.20919970
        z: 1.20920010
    position_noise:
        x: 0.00010000
        y: 0.00010000
        z: 0.00010000
    rotation_noise:
        x: 0.00001000
        y: 0.00001000
        z: 0.00001000
'''
# multi cursor magic > regex

# x y z angle id
tags = [
[15.079,0.246,1.356,120.000,1],
[16.185,0.884,1.356,120.000,2],
[16.579,4.983,1.451,180.000,3],
[16.579,5.548,1.451,180.000,4],
[14.701,8.204,1.356,-90.000,5],
[1.841,8.204,1.356,-90.000,6],
[-0.013,5.548,1.451,0.000,7],
[-0.013,4.983,1.451,0.000,8],
[0.356,0.884,1.356,60.000,9],
[1.462,0.246,1.356,60.000,10],
[11.905,3.713,1.321,-60.000,11],
[11.905,4.498,1.321,60.000,12],
[11.220,4.105,1.321,180.000,13],
[5.321,4.105,1.321,0.000,14],
[4.641,4.498,1.321,120.000,15],
[4.641,3.713,1.321,-120.000,16]
]

data = dict(
    tags = [
    ]
)

# MUST BE CAPITALIZED

r = Rotation.from_euler("XY", (90, -90), degrees=True)
print(r.as_rotvec())
X_angle_offset = 90
POSITION_NOISE = 0.0005
ROTATION_NOISE = 0.004

for tag in tags:
    #r = Rotation.from_euler("xy", (90, -90), degrees=True)
    d = dict()
    d["id"] = tag[-1]
    d["size"] = 0.16510000
    d['pose'] = dict()
    pose = d['pose']
    pose['position'] = dict(x=tag[0], y=tag[1], z=tag[2])
    
    print(f"using angles X {X_angle_offset} Y {tag[-2]+270} Z 0")

    r = Rotation.from_euler("XY", (X_angle_offset, tag[-2]+90), degrees=True)
    vec = r.as_rotvec()
    
    pose['rotation'] = dict(x=float(vec[0]), y=float(vec[1]), z=float(vec[2]))
    pose["position_noise"] = dict(x=POSITION_NOISE, y=POSITION_NOISE, z=POSITION_NOISE)    
    pose["rotation_noise"] = dict(x=ROTATION_NOISE, y=ROTATION_NOISE, z=ROTATION_NOISE)
    data['tags'].append(d)

    #r = Rotation.from_quat([0.4469983, -0.4469983, -0.7240368, 0.2759632])
    #print(r.as_rotvec())

with open('data.yml', 'w') as outfile:
    yaml.dump(data, outfile, default_flow_style=False)

