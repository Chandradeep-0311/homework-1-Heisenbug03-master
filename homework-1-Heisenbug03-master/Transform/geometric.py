import math

import numpy as np

from .interpolation import interpolation


class Geometric:
    def __init__(self):
        pass

    def forward_point(self, pt, thetha):  # Function used to forward rotate the corner points
        npt = np.array((2, 1))
        npt[0] = int(pt[0] * math.cos(thetha) - pt[1] * math.sin(thetha))
        npt[1] = int(pt[0] * math.sin(thetha) + pt[1] * math.cos(thetha))
        return npt

    def nearest_neighbour(self, image, pt):  # Function to perform nearest neighbour interpolation
        pt1 = (math.floor(pt[0]), math.floor(pt[1]))
        if 0 <= pt1[0] < image.shape[0] and 0 <= pt1[1] < image.shape[1]:
            I = image[pt1]
        else:
            I = 0
        return I

    def forward_rotate(self, image, theta):
        """Computes the forward rotated image by an angle theta
                image: input image
                theta: angle to rotate the image by (in radians)
                return the rotated image"""
        size = np.shape(image)  # since origin is (0,0), the corners are manually defined below
        pt1 = (0, 0)
        pt2 = (0, size[1])
        pt4 = (size[0], 0)
        pt3 = (size[0], size[1])

        npts1 = self.forward_point(pt1, theta)  # rotated corners using the forward rotate function defined above
        npts2 = self.forward_point(pt2, theta)
        npts3 = self.forward_point(pt3, theta)
        npts4 = self.forward_point(pt4, theta)

        maximum_x = max(npts1[0], npts2[0], npts3[0], npts4[0])
        minimum_x = min(npts1[0], npts2[0], npts3[0], npts4[0])
        maximum_y = max(npts1[1], npts2[1], npts3[1], npts4[1])
        minimum_y = min(npts1[1], npts2[1], npts3[1], npts4[1])
        rotated_rows = abs(maximum_x - minimum_x)
        rotated_cols = abs(maximum_y - minimum_y)
        rotated_image = np.zeros((rotated_rows, rotated_cols))
        for i in range(size[0]):
            for j in range(size[1]):
                i_t = int(i * math.cos(theta) - j * math.sin(theta))
                j_t = int(i * math.sin(theta) + j * math.cos(theta))
                i_n = abs(i_t - minimum_x)
                j_n = abs(j_t - minimum_y)
                if 0 <= i_n < rotated_image.shape[0] and 0 <= j_n < rotated_image.shape[1]:
                    rotated_image[i_n, j_n] = image[i, j]
        return rotated_image

    def reverse_rotation(self, rotated_image, theta, origin, original_shape):
        """Computes the reverse rotated image by an angle theta
                rotated_image: the rotated image from previous step
                theta: angle to rotate the image by (in radians)
                Origin: origin of the original image with respect to the rotated image
                Original shape: shape of the orginal image
                return the original image"""
        size = np.shape(rotated_image)
        reversed_image = np.zeros(original_shape)
        for i_n in range(size[0]):
            for j_n in range(size[1]):
                i_t = i_n - origin[0]
                j_t = j_n - origin[1]
                i = int(i_t * math.cos(theta) + j_t * math.sin(theta))
                j = int(j_t * math.cos(theta) - i_t * math.sin(theta))
                if 0 <= i < reversed_image.shape[0] and 0 <= j < reversed_image.shape[1]:
                    reversed_image[i, j] = rotated_image[i_n, j_n]

        return reversed_image

    def rotate(self, image, theta, interpolation_type):
        """Computes the forward rotated image by an angle theta using interpolation
                image: the input image
                theta: angle to rotate the image by (in radians)
                interpolation_type: type of interpolation to use (nearest_neighbor, bilinear)
                return the original image"""
        size = np.shape(image)
        pt1 = (0, 0)
        pt2 = (0, size[1])
        pt4 = (size[0], 0)
        pt3 = (size[0], size[1])

        npts1 = self.forward_point(pt1, theta)
        npts2 = self.forward_point(pt2, theta)
        npts3 = self.forward_point(pt3, theta)
        npts4 = self.forward_point(pt4, theta)

        maximum_x = max(npts1[0], npts2[0], npts3[0], npts4[0])
        minimum_x = min(npts1[0], npts2[0], npts3[0], npts4[0])
        maximum_y = max(npts1[1], npts2[1], npts3[1], npts4[1])
        minimum_y = min(npts1[1], npts2[1], npts3[1], npts4[1])

        rotated_rows = abs(maximum_x - minimum_x)
        rotated_cols = abs(maximum_y - minimum_y)
        rotated_image = np.zeros((rotated_rows, rotated_cols))

        o = (-minimum_x, -minimum_y)

        for i_n in range(rotated_rows):
            for j_n in range(rotated_cols):
                i_t = i_n - o[0]
                j_t = j_n - o[1]
                i = int(i_t * math.cos(theta) + j_t * math.sin(theta))
                j = int(j_t * math.cos(theta) - i_t * math.sin(theta))

                # bilinear interpolation
                if interpolation_type == 'bilinear':
                    blp1 = (math.floor(i), math.floor(j))
                    blp2 = (math.ceil(i), math.floor(j))
                    blp3 = (math.floor(i), math.ceil(j))
                    blp4 = (math.ceil(i), math.ceil(j))

                    I1 = None
                    I2 = None
                    I3 = None
                    I4 = None
                    if 0 <= blp1[0] < image.shape[0] and 0 <= blp1[1] < image.shape[1]:
                        I1 = image[blp1]

                    if 0 <= blp2[0] < image.shape[0] and 0 <= blp2[1] < image.shape[1]:
                        I2 = image[blp2]

                    if 0 <= blp3[0] < image.shape[0] and 0 <= blp3[1] < image.shape[1]:
                        I3 = image[blp3]

                    if 0 <= blp4[0] < image.shape[0] and 0 <= blp4[1] < image.shape[1]:
                        I4 = image[blp4]

                    if I1 is not None and I2 is not None and I3 is not None and I4 is not None:
                        if int(i) == i and int(j) == j:
                            rotated_image[i_n, j_n] = image[i, j]
                            continue

                        rotated_image[i_n, j_n] = interpolation().bilinear_interpolation(blp1, blp2, blp3, blp4, I1, I2,
                                                                                         I3, I4, (i, j))

                # nearest neighbour
                if interpolation_type == 'nearest_neighbor':
                    rotated_image[i_n, j_n] = self.nearest_neighbour(image,
                                                                     (i, j))  # refer to function defined on the top

        return rotated_image
