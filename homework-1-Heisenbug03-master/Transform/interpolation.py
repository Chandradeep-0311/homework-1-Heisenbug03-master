class interpolation:

    def linear_interpolation(self, x1, x2, x, i1, i2):
        """Computes the linear interpolation value at some iD location x between two 1D points (Pt1 and Pt2).

        There are no arguments defined in the function definition on purpose. It is left upto the student to define any requierd arguments.
        Please change the signature of the function and add the arguments based on your implementation.

        The function ideally takes two 1D points Pt1 and Pt2, and their intensitites I(Pt1), I(Pt2).
        return the interpolated intensity value (I(x)) at location x """

        # Write your code for linear interpolation here
        # 2 points and their intensities are taken to define the linear interpolation function

        i = (i1 * (x2 - x) / (x2 - x1)) + (i2 * (x - x1) / (x2 - x1))
        return i

    def bilinear_interpolation(self, pt1, pt2, pt3, pt4, i1, i2, i3, i4, pt):
        """Computes the bi linear interpolation value at some 2D location x between four 2D points (Pt1, Pt2, Pt3, and Pt4).

        There are no arguments defined in the function definition on purpose. It is left upto the student to define any requierd arguments.
        Please change the signature of the function and add the arguments based on your implementation.

        The function ideally takes four 2D points Pt1, Pt2, Pt3, and Pt4, and their intensitites I(Pt1), I(Pt2), I(Pt3), and I(Pt4).
        return the interpolated intensity value (I(x)) at location x """

        # Write your code for bilinear interpolation here
        # for bilinear interpolation , 4 points and their intensities are defined
        midpoint12 = (pt[
            0])  #since points and 1 and 2 assumed to be adjacent, Their x cordinate of required point is used as point between them
        midpoint34 = (pt[0])  # similarly for 3 and 4, x cordinate is also used
        i12 = self.linear_interpolation(pt1[0], pt2[0], midpoint12, i1,
                                        i2)  # linear interpolation is performed horizontally b/w 1 and 2
        i34 = self.linear_interpolation(pt3[0], pt4[0], midpoint12, i3,
                                        i4)  # linear interpolation is performed horizontally b/w 3 and 4
        i = self.linear_interpolation(pt1[1], pt3[1], pt[1], i12, i34)

        # Recall that bilinear interpolation performs linear interpolation three times
        # Please reuse or call linear interpolation method three times by passing the appropriate parameters to compute this task

        return i


