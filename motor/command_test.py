while True:
    cmd = raw_input("Command, a/s/w: ")
    motor = cmd[0]
    vel = cmd[1:]
    if cmd[0] == "a":
        print "right motor " + vel
    elif cmd == 's':
        print "left motor " + vel
    elif cmd == "w":
        print "both motors " + vel
    else:
        print "incorrect command"