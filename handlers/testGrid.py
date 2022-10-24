from grid import Grid, Point
import numpy as np


class testGrid:
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
        print(real_basis)

        virt = {
            "A": Point(570, 1322),
            "B": Point(570, 570),
            "C": Point(1507, 570),
            "D": Point(1507, 1322)
        }

        a11 = 937 / 1.087
        a12 = 0
        a21 = 0
        a22 = -752 / 1.067

        A = np.array([[a11, a12], [a21, a22]])
        A_1 = np.linalg.inv(A)
        self.A_1 = A_1
        self.A = -A_1 @ np.array([570, 1322])

    def transform(self, point):
        print(point)
        point = np.array([point.getX(), point.getY()])
        return self.A_1 @ np.array(point) + self.A + np.array([0.541, 0.534])



if __name__ == "__main__":
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
    print(real_basis)

    virt = {
        "A": Point(570, 1322),
        "B": Point(570, 570),
        "C": Point(1507, 570),
        "D": Point(1507, 1322)
    }

    A = np.array([[real_basis['D'].getX(), real_basis['D'].getY()], [real_basis['B'].getX(), real_basis['B'].getY()]])
    B = np.array([[virt['D'].getX()-virt['A'].getX()], [virt['B'].getX()-virt['A'].getX()]])
    X = np.linalg.inv(A).dot(B)
    print(937 / 1.087, -752 / 1.067)
    print(X)
    A = np.array([[real_basis['D'].getX(), real_basis['D'].getY()], [real_basis['B'].getX(), real_basis['B'].getY()]])
    B = np.array([[virt['D'].getY() - virt['A'].getY()], [virt['B'].getY() - virt['A'].getY()]])
    X = np.linalg.inv(A).dot(B)
    print(X)



