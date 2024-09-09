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
    sleep(0.041)
    print(arlo.go_diff(h_left, h_right, 1, 0))
    sleep(sekunder)


sekunder_drej = 0.55
sekunder_lige = 3
hastighed_kør_r = 63
hastighed_kør_l = 63
hastighed_drej_l = 35
hastighed_drej_r = 35

print("Jytte, mindre press på speederen! ...")
sleep(1)

for rotation in range(1, 4):

    lige_ud(sekunder_lige, hastighed_kør_l, hastighed_kør_r)
    print(arlo.stop())
    sleep(3.041)
    drej(sekunder_drej, hastighed_drej_l, hastighed_drej_r)
    print(arlo.stop())
    sleep(3.041)
    lige_ud(sekunder_lige, hastighed_kør_l, hastighed_kør_r)
    print(arlo.stop())
    sleep(3.041)
    drej(sekunder_drej, hastighed_drej_l, hastighed_drej_r)
    print(arlo.stop())
    sleep(3.041)
    lige_ud(sekunder_lige, hastighed_kør_l, hastighed_kør_r)
    print(arlo.stop())
    sleep(3.041)
    drej(sekunder_drej, hastighed_drej_l, hastighed_drej_r)
    print(arlo.stop())
    sleep(3.041)
    lige_ud(sekunder_lige, hastighed_kør_l, hastighed_kør_r)
    print(arlo.stop())
    sleep(3.041)
    drej(sekunder_drej, hastighed_drej_l, hastighed_drej_r)
    print(arlo.stop())
    sleep(3.041)
print(arlo.stop())
