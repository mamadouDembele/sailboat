import numpy as np

M1=np.ones((20,20))
M2=np.ones((20,20))

def main_boat1_90(M):
	for i in range(0,20):
		for j in range(0,20):
			if i==0 and j%2==0:
				M[i][j]=0
			if i==19 and j%2==1:
				M[i][j]=0
	M[0][0]=1
	return M

def main_boat1_30(M):
	for i in range(0,20):
		for j in range(0,20):
			if i==18 and j%2==1:
				M[i][j]=0
	M[0][0]=1
	return M

if __name__ == '__main__':
	print(main_boat1_90(M1))
	
	print("\n----------------------\n")

	print(main_boat1_30(M2))