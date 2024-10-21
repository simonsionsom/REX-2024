sekunder_drej = 0.55
sekunder_lige = 3
hastighed_kør_r = 63
hastighed_kør_l = 63
hastighed_drej_l = 35
hastighed_drej_r = 35


from time import sleep
import PreMade.robot as robot


arlo = robot.Robot()

print("Kører lige ud: ")
print(arlo.go_diff(hastighed_kør_l, hastighed_kør_r, 1, 1))
sleep(3)
print(arlo.stop())
sleep(0.7)
print("Drejer med hastighederne: ")
print("Venstre: {h_left}, Højre: {h_right}")
print("I {sekunder_drej} sekunder")
sleep(1)
print(arlo.go_diff(hastighed_drej_l, hastighed_drej_r, 1, 0))
sleep(float(1.2))

