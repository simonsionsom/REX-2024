import numpy as np


X = 146.0
F = [None] * 10
x = [None] * 10
Z = [None] * 10
next = 0
def Focal_length(next, f,x,Z,X):
  f[next] = (x[next]*Z[next])/X
  return f

while True:
  print("Hvad er længden til objekt? :")
  Z[next] = float(input())
  print('Hvad er lille x? :')
  x[next] = float(input())
  print('udregner: ')
  Focal_length(next,F,x,Z,X)
  print('Her er de udregnede længer so far: \n', F )
  if next >= 9 or Z[next] == 0 :
    break
  else :
    next = next + 1
print('Skal det gemmes? j/n?')
svar = input()
if svar == 'j':
  with open("arrays_output.txt", "w") as file:
      # Write each array to a new line in the file
      file.write("Array F: " + " ".join(map(str, F)) + "\n")
      file.write("Array x: " + " ".join(map(str, x)) + "\n")
      file.write("Array Z: " + " ".join(map(str, Z)) + "\n")
      print("Arrays saved to arrays_output.txt")