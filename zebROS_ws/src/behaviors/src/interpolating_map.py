class InterpolatingMap:
    def __init__(self):
        self.container = {}

    def insert(self, key, value):
        self.container[key] = value

    def __getitem__(self, key):
        keys = sorted(self.container.keys())

        if key >= keys[-1]:
            return self.container[keys[-1]]

        if key <= keys[0]:
            return self.container[keys[0]]

        upper_key = next(k for k in keys if k > key)
        lower_key = keys[keys.index(upper_key) - 1]

        delta = (key - lower_key) / (upper_key - lower_key)
        return delta * self.container[upper_key] + (1 - delta) * self.container[lower_key]

    def clear(self):
        self.container.clear()

if __name__ == "__main__":
    interpolating_map = InterpolatingMap()
    interpolating_map.insert(0, 1231)
    interpolating_map.insert(1, 5353)
    interpolating_map.insert(3, 13131)
    print(interpolating_map[1])