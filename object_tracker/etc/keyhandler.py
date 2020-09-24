import string


class KeyHandler:
    def __init__(self):
        self.keyHandler = dict()
        self.exitFlag = False

    # register a key and handler
    def setKeyHandler(self, key, handler):
        # register lower cases only unconditioanlly
        if key in string.ascii_uppercase:
            key = key.lower()

        # set key and handler pair
        self.keyHandler[key] = handler

    # handle key events
    def getKeyHandler(self, key):
        return self.keyHandler[key]

    # enable exit flag
    def enableExitFlag(self):
        self.exitFlag = True

    def processKeyHandler(self, keyVal, *args):
        # ascii value to character
        key = chr(keyVal)

        # check if key is ascii
        if not (key in string.ascii_letters):
            return False

        # in case that key is one of alphabet, handle upper cases as the same as lower case..
        if key in string.ascii_uppercase:
            key = key.lower()

        if(key in self.keyHandler.keys()):
            func = self.getKeyHandler(key)
            func(*args)

        return self.exitFlag
