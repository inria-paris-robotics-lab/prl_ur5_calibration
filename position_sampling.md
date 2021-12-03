Position sampling
===

# How the camera position were choose for calibration : 

The camera position were sampled from : 
```
interest_angles = [0, np.pi/6, -np.pi/6, np.pi/4, -np.pi/4, np.pi/3, -np.pi/3, np.pi/2, -np.pi/2]
sample_rpy = []
for r in interest_angles:
    for p in interest_angles:
        for y in interest_angles:
            sample_rpy.append(np.array([r, p, y]))
```

From this positions, the ones that were considered as achievable by the path planner (RRT set for 2s) are : 

```
achievable_pos_index = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 16, 18, 19, 20, 21, 22, 23, 24, 25, 26, 36, 37, 38, 39, 40, 41, 44, 54, 55, 56, 57, 58, 89, 100, 101, 103, 105, 107, 119, 121, 123, 125, 135, 137, 139, 141, 143, 162, 163, 164, 165, 166, 167, 168, 169, 170, 178, 180, 181, 182, 183, 184, 185, 187, 199, 201, 203, 205, 217, 219, 221, 265, 267, 269, 279, 281, 283, 285, 299, 301, 303, 327, 331, 343, 345, 347, 349, 363, 365, 367, 381, 383, 385, 431, 443, 445, 447, 449, 465, 467, 493, 511, 529, 547, 575, 593]
```

From those positions, the one were the QR code was read successfully were :
```
readable_pos = [ 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 , 21 , 22 , 23 , 24 , 25 , 27 , 28 , 29 , 30 , 31 , 37 , 39 , 41 , 42 , 43 , 46 , 52 , 53 , 54 , 55 , 56 , 57 , 58 , 59 , 60 , 61 , 62 , 63 , 64 , 65 , 67 , 68 , 77 , 78 , 87 , 88 , 89 , 90 , 105]
```

