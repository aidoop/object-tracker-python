import sys
import cv2


class SingletonInstane:
    __instance = None

    @classmethod
    def __getInstance(cls):
        return cls.__instance

    @classmethod
    def instance(cls, *args, **kargs):
        cls.__instance = cls(*args, **kargs)
        cls.instance = cls.__getInstance
        return cls.__instance


class ObjectTypeCheck:
    @staticmethod
    def check_value_available(var):
        if var is None:
            return False
        return True


class ObjectTrackerErrMsg:
    @staticmethod
    def check_value_none(var, varname):
        if var is None:
            print("Error: " + varname + " is not allocated..", file=sys.stderr)
            return False
        return True


class PrintMsg:
    @staticmethod
    def print_error(*args, **kwargs):
        return print(*args, **kwargs, file=sys.stderr)

# Info Text


class DisplayInfoText:
    def __init__(self, font, startPos, image_width, image_height):
        self.font = font
        self.startPos = startPos
        self.text = ""
        self.image_width = image_width
        self.image_height = image_height

    def draw(self, image):
        if self.text == "":
            return

        font_scale = 3 if self.image_width <= 1920 else 5
        font_thickness = 2 if self.image_width <= 1920 else 4

        cv2.putText(
            image,
            self.text,
            self.startPos,
            self.font,
            font_scale,
            (0, 255, 0),
            font_thickness,
            cv2.LINE_AA,
        )

    def set_info_text(self, text):
        self.text = text
