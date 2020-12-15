import  os
import  rospi.teleop

if "DISPLAY" in os.environ and os.environ["DISPLAY"] != "":
    print("Using PyGame interface")
    import rospi.teleop.xpygame
    win   = rospi.teleop.xpygame.PygameInterface()
else:
    print("Using curses interface")
    import rospi.teleop.xcurses
    win   = rospi.teleop.xcurses.CursesInterface()

rospi.teleop.run(win)
