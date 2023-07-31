import NAR
import json

while True:
    try:
        user_input = input()
    except:
        break
    ret = NAR.AddInput(user_input, Print=False)
    json_string = json.dumps(ret)
    print(json_string)
