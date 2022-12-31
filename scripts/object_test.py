import yaml

class test:
    def __init__(self):
        self.pid_dict = {}
        self.part_names = ['q', 'w', 'e', 'r']
        for part_name in self.part_names:
            self.pid_dict[part_name] = {}
            self.pid_dict[part_name]['p'] = 0
            self.pid_dict[part_name]['i'] = 0
            self.pid_dict[part_name]['d'] = 0

    

test1 = test()
test2 = test()
filename = "test_pid.yaml"
with open(filename, "w") as fp1:
    yaml.dump(test1.pid_dict, fp1)

with open(filename, "r") as fp2:
    loaded = yaml.full_load(fp2)
    print(loaded)