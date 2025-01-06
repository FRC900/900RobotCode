from scipy.spatial.transform import Rotation
import yaml

# data = dict(
#     tags = [
#         dict(
#             id = 1,
#             size = 0.165
#         ),
#         dict(
#             id = 2,
#             size = 0.165
#         )
#     ]
# )

# with open('data.yml', 'w') as outfile:
#     yaml.dump(data, outfile, default_flow_style=False)
# '''     
# tags:
# - id: 7
#     size: 0.16510000
#     pose:
#     position:
#         x: -0.01270000
#         y: 5.54786800
#         z: 1.45110200
#     rotation:
#         x: 1.20919970
#         y: 1.20919970
#         z: 1.20920010
#     position_noise:
#         x: 0.00010000
#         y: 0.00010000
#         z: 0.00010000
#     rotation_noise:
#         x: 0.00001000
#         y: 0.00001000
#         z: 0.00001000
# '''
# # multi cursor magic > regex

# x y z angle id
tags = [

#[15.079,0.246,1.356,120.000,1]
]

with open("2025tags.csv", "r") as f:
    lines = f.read().split("\n")

lines = lines[1:]
lines.pop()
for tag in lines:
    
    # there format is, [id, x, y, z, z rot, y rot 
    # our format is    [x, y, z, y rot, id
    # no z rot for now
    tag_id, x, y, z, z_rot, y_rot = tag.split(",")
    tag_id = int(tag_id)
    x = float(x)
    y = float(y)
    z = float(z)
    z_rot = int(z_rot)
    y_rot = int(y_rot)

    x *= 0.0254 # 254! 
    y *= 0.0254
    z *= 0.0254
    print(f"tag id {tag_id}, x {x}, y {y}, {z}, z_rot {z_rot}, y rot {y_rot}")

    tags.append([x, y, z, y_rot, tag_id])

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

