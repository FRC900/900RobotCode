from scipy.spatial.transform import Rotation
import yaml

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
    # our format is    [x, y, z, z_rot, y rot, id

    # their Z rot is our Y rot 
    # their X rot is our... Z rot 
    # no z rot for now
    tag_id, x, y, z, y_rot, z_rot = tag.split(",")
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

    tags.append([x, y, z, z_rot, y_rot, tag_id])

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
    
    print(f"using angles X {X_angle_offset} Y {tag[-2]+270} Z {tag[3]}")

    r = Rotation.from_euler("XYZ", (X_angle_offset, tag[-2]+90, tag[3]), degrees=True)
    vec = r.as_rotvec()
    
    pose['rotation'] = dict(x=float(vec[0]), y=float(vec[1]), z=float(vec[2]))
    pose["position_noise"] = dict(x=POSITION_NOISE, y=POSITION_NOISE, z=POSITION_NOISE)    
    pose["rotation_noise"] = dict(x=ROTATION_NOISE, y=ROTATION_NOISE, z=ROTATION_NOISE)
    data['tags'].append(d)

    #r = Rotation.from_quat([0.4469983, -0.4469983, -0.7240368, 0.2759632])
    #print(r.as_rotvec())

with open('data.yml', 'w') as outfile:
    yaml.dump(data, outfile, default_flow_style=False)

