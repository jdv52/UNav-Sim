import airsim
import cv2
import matplotlib.pyplot as plt
import numpy as np
import time

def main():
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)
    print("API Control enabled: %s" % client.isApiControlEnabled())

    plt.ion()

    fig, (ax1, ax2) = plt.subplots(1, 2)
    fig.suptitle('Sonar Data')
    
    ax1.set_title('Polar View')
    img1 = ax1.imshow(np.zeros((100, 100)), cmap='gray', vmin=0, vmax=255)

    ax2.set_title('Sonar Image')
    img2 = ax2.imshow(np.zeros((100, 100)), cmap='gray', vmin=0, vmax=255)
    

    while True:
        response = client.getSonarData("Sonar", "RovSimple")
        data_arr = np.array(response.image).reshape(response.data_shape[0], response.data_shape[1])
    
        image = cv2.normalize(data_arr, None, 0, 255, cv2.NORM_MINMAX)
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        image = cv2.flip(image, -1)

        range_bins, angle_bins = image.shape
        output_size = 500
        center = output_size // 2
        min_range = 0.1
        max_range = 5

        fov = np.deg2rad(120)

        y, x = np.indices((output_size, output_size))
        dx = (x - center)
        dy = (y - center)
       
        thing = min_range / max_range * output_size / 2
        r = (np.sqrt(dx ** 2 + dy**2))
        valid = r > thing
        r[valid] = (r[valid] - thing) / ((output_size / 2) - thing) * range_bins
        r[~valid] = -1
        theta = np.arctan2(dy, dx) + (np.pi / 2)

        map_y = r
        map_x = (theta + fov / 2) / fov * (angle_bins - 1)

        valid = (r > 0) & (r < range_bins) & (theta >= -fov/2) & (theta <= fov/2)
        map_x[~valid] = -1
        map_y[~valid] = -1

        output = cv2.remap(
            image,
            map_x.astype(np.float32),
            map_y.astype(np.float32),
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=255
        )

        img1.set_data(output)

        img2.set_data(image)

        plt.draw()
        plt.pause(0.1)

if __name__ == "__main__":
    main()