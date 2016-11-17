while True:
    cmd = raw_input("Command, a/s/w")
    if cmd == "a":
        print "right motor"
    elif cmd == 's':
        print "left motor"
    elif cmd == "w":
        print "both motors"
    else:
        print "incorrect command"