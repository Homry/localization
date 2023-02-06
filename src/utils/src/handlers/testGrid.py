from grid import Grid, Point
import numpy as np


class testGrid:
    def __init__(self, config, virtual_points):
        print(virtual_points)
        real = {
            "A": Point(0.541, 0.534),
            "B": Point(0.541, 1.601),
            "C": Point(1.628, 1.601),
            "D": Point(1.628, 0.534)
        }
        markers = config["markers"]
        real = {
            "A": Point(markers[0][0], markers[0][1]),
            "B": Point(markers[1][0], markers[1][1]),
            "C": Point(markers[2][0], markers[2][1]),
            "D": Point(markers[3][0], markers[3][1])
        }
        real_basis = {
            "A": Point(0.541 - real["A"].getX(), 0.534 - real["A"].getY()),
            "B": Point(0.541 - real["A"].getX(), 1.601 - real["A"].getY()),
            "C": Point(2.171 - real["A"].getX(), 1.601 - real["A"].getY()),
            "D": Point(1.628 - real["A"].getX(), 0.534 - real["A"].getY())
        }
        real_basis = {
            "A": Point(real["A"].getX() - real["A"].getX(), real["A"].getY() - real["A"].getY()),
            "B": Point(real["B"].getX() - real["A"].getX(), real["B"].getY() - real["A"].getY()),
            "C": Point(real["C"].getX() - real["A"].getX(), real["C"].getY() - real["A"].getY()),
            "D": Point(real["D"].getX() - real["A"].getX(), real["D"].getY() - real["A"].getY())
        }

        virt = {
            "A": Point(570, 1322),
            "B": Point(570, 570),
            "C": Point(1507, 570),
            "D": Point(1507, 1322)
        }
        virt = {
            "A": Point(virtual_points[0][0], virtual_points[0][1]),
            "B": Point(virtual_points[1][0], virtual_points[1][1]),
            "C": Point(virtual_points[2][0], virtual_points[2][1]),
            "D": Point(virtual_points[3][0], virtual_points[3][1])
        }

        A = np.array([[real_basis['D'].getX(), real_basis['D'].getY()], [real_basis['B'].getX(), real_basis['B'].getY()]])
        B = np.array([[virt['D'].getX() - virt['A'].getX()], [virt['B'].getX() - virt['A'].getX()]])
        X_1 = np.linalg.inv(A).dot(B)
        A = np.array(
            [[real_basis['D'].getX(), real_basis['D'].getY()], [real_basis['B'].getX(), real_basis['B'].getY()]])
        B = np.array([[virt['D'].getY() - virt['A'].getY()], [virt['B'].getY() - virt['A'].getY()]])
        X_2 = np.linalg.inv(A).dot(B)
        A = np.array([[X_1[0][0], 0], [0, X_2[1][0]]])
        A_1 = np.linalg.inv(A)
        self.A_1 = A_1
        self.A = -A_1 @ np.array([virt['A'].getX(), virt['A'].getY()])
        self.real = real
    def transform(self, point):
        point = np.array([point.getX(), point.getY()])
        return self.A_1 @ np.array(point) + self.A + np.array([self.real['A'].getX(), self.real['A'].getY()])


class test:
    def __init__(self):
        real = {
            "A": Point(0.541, 0.534),
            "B": Point(0.541, 1.601),
            "C": Point(1.628, 1.601),
            "D": Point(1.628, 0.534)
        }

        real_basis = {
            "A": Point(0.541 - real["A"].getX(), 0.534 - real["A"].getY()),
            "B": Point(0.541 - real["A"].getX(), 1.601 - real["A"].getY()),
            "C": Point(2.171 - real["A"].getX(), 1.601 - real["A"].getY()),
            "D": Point(1.628 - real["A"].getX(), 0.534 - real["A"].getY())
        }

        virt = {
            "A": Point(570, 1322),
            "B": Point(570, 570),
            "C": Point(1507, 570),
            "D": Point(1507, 1322)
        }

        A = np.array(
            [[real_basis['D'].getX(), real_basis['D'].getY()], [real_basis['B'].getX(), real_basis['B'].getY()]])
        B = np.array([[virt['D'].getX() - virt['A'].getX()], [virt['B'].getX() - virt['A'].getX()]])
        X_1 = np.linalg.inv(A).dot(B)
        A = np.array(
            [[real_basis['D'].getX(), real_basis['D'].getY()], [real_basis['B'].getX(), real_basis['B'].getY()]])
        B = np.array([[virt['D'].getY() - virt['A'].getY()], [virt['B'].getY() - virt['A'].getY()]])
        X_2 = np.linalg.inv(A).dot(B)
        A = np.array([[X_1[0][0], 0], [0, X_2[1][0]]])
        A_1 = np.linalg.inv(A)
        self.A_1 = A_1
        self.A = -A_1 @ np.array([virt['A'].getX(), virt['A'].getY()])
        self.real = real

    def transform(self, point):
        point = np.array([point.getX(), point.getY()])
        return self.A_1 @ np.array(point) + self.A + np.array([self.real['A'].getX(), self.real['A'].getY()])


if __name__ == '__main__':
   t = test()
   a = Point(0, 0)
   print(t.transform(a))




