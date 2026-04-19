# This class has been modified in order to get functionality of linked hashmap
# Due to; dictionaries don't allow indexing and this project's drone manager needs to know order of inserted items which
# might have key value pairs; as nodes, targets, other drones etc... in this scenario.


class ordered_dict(dict):
    def __init__(self, *args, **kwargs):
        dict.__init__(self, *args, **kwargs)
        self._index = self.keys()

    def __setitem__(self, key, value):
        dict.__setitem__(self, key, value)
        if key in self._index:
            self._index.remove(key)
        self._index.append(key)

    def __delitem__(self, key):
        dict.__delitem__(self, key)
        self._index.remove(key)

    def index(self):
        return self._index[:]

    def indexed_items(self):
        return [(key, self[key]) for key in self._index]
