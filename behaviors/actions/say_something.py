import sys

def SaySomething(message):
    """
    A custom action that prints a message and returns success.
    """
    print(f"{sys._getframe().f_code.co_name}: {message}")
    print("wait a minute")
    return False
