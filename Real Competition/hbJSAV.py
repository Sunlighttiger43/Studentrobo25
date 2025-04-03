#HAPPY BIRTHDAY, MR SAVAGE!
from sr.robot3 import *
from time import sleep

r = Robot()
p = r.power_board.piezo

s = 2.0

def quaver(freq):
    p.buzz(freq,0.1*s,blocking=True)
    sleep(0.025*s)

def crotchet(freq):
    p.buzz(freq,0.2*s,blocking=True)
    sleep(0.05*s)

def minim(freq):
    p.buzz(freq,0.45*s,blocking=True)
    sleep(0.05*s)

def restCrotchet():
    sleep(0.25*s)

def restMinim():
    sleep(0.5*s)

def playHappyBirthday():
    #HAPPY BIRTHDAY
    quaver(392)
    quaver(392)
    crotchet(440)
    crotchet(392)
    crotchet(523)
    minim(494)
    quaver(392)
    quaver(392)
    crotchet(440)
    crotchet(392)
    crotchet(587)
    minim(523)
    quaver(392)
    quaver(392)
    crotchet(784)
    crotchet(659)
    crotchet(523)
    crotchet(494)
    crotchet(440)
    quaver(698)
    quaver(698)
    crotchet(659)
    crotchet(523)
    crotchet(587)
    minim(523)

def main():
    for i in range(10):
        playHappyBirthday()
        sleep(1)

if __name__ == '__main__':
    main()