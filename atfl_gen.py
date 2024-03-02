import json

out = []
for i in range(16):
    out.append({})

tags = list(range(1, 17))
tags_part = {}

with open("partial.json", "r") as f:
    tags_part = json.load(f)
    for tag in tags_part['tags']:
        tags.remove(tag['ID'])
        out[tag["ID"] - 1] = tag

for tag in tags:
    out[tag - 1] = {
        "ID": tag,
        "pose": {
            "translation": {
                "x": 0.00,
                "y": 0.00,
                "z": 0.00
            },
            "rotation": {
                "quaternion": {
                    "W": 1.00,
                    "X": 0.0,
                    "Y": 0.0,
                    "Z": 0.00
                }
            }
        }
    }

with open('out.json', "w") as f:
    json.dump({
        "tags": out,
        "field": tags_part["field"]
        }, f)
