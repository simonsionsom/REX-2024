from time import sleep
import robot


arlo = robot.Robot()

def lige_ud(sekunder, h_left, h_right): 
    print("Kører lige ud: ")
    print(arlo.go_diff(h_left, h_right, 1, 1))
    sleep(sekunder)
  




def drej(sekunder, h_left, h_right):
    print("Drejer med hastighederne: ")
    print("Venstre: {h_left}, Højre: {h_right}")
    print("I {sekunder} sekunder")
    print(arlo.go_diff(h_left, h_right, 0, 1))
    sleep(sekunder)


print("Jytte, mindre press på speederen! ...")
sleep(0.041)
lige_ud(0.5, 63, 64)
sleep(0.041)
drej(0.5, 63, 63.5)
sleep(0.041)
sleep(0.041)
lige_ud(0.5, 63, 64)
sleep(0.041)
drej(0.5, 63, 63.5)
sleep(0.041)
sleep(0.041)
lige_ud(0.5, 63, 64)
sleep(0.041)
drej(0.5, 63, 63.5)
sleep(0.041)
sleep(0.041)
lige_ud(0.5, 63, 64)
sleep(0.041)
drej(0.5, 63, 63.5)
sleep(0.041)
print(arlo.stop())
