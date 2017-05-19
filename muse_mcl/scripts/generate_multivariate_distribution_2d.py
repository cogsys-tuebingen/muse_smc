import numpy
import math
import yaml
import matplotlib.pyplot as plt

def plot(data, markerDiameter = .1, markerLength = .01):
    for row in data:
        plt.plot(row[0], row[1], 'ro')
        phi = row[2]
        xxs = [(row[0]), row[0] + markerLength * math.cos(phi)]
        yys = [(row[1]), row[1] + markerLength * math.sin(phi)]
        plt.plot(xxs, yys, 'k-')

    # draw diagonal line from (70, 90) to (90, 200)


    plt.show()


def vector_to_dictionary(name, arr):
    dictionary = {}
    dictionary['rows'] = len(arr)
    dictionary['cols'] = 1
    dictionary['data'] = []
    for i in range(dictionary['rows']):
        dictionary['data'].append(float(arr[i]))

    return [name, dictionary]

def matrix_to_dictionary(name, arr):
    dictionary = {}
    dictionary['rows'] = len(arr)
    dictionary['cols'] = len(arr[0])
    dictionary['data'] = []
    for i in range(dictionary['rows']):
        for j in range(dictionary['cols']):
            dictionary['data'].append(float(arr[i][j]))

    return [name, dictionary]

def create_distribution(mu, sigma, n, path):
    samples = numpy.random.multivariate_normal(mu, sigma, n)
    est_mu = numpy.mean(samples, axis=0)
    est_sigma = numpy.cov(samples, rowvar=False)
    eigen_values, eigen_vectors = numpy.linalg.eig(est_sigma)

    # prepare yaml
    dump = {}
    entry_cov = matrix_to_dictionary('covariance', est_sigma)
    entry_mean = vector_to_dictionary('mean', est_mu)
    entries_data = []
    for row in samples:
        entry = vector_to_dictionary("entry", row)
        entries_data.append(entry[1])
    entry_eigen_values = vector_to_dictionary("eigen_values", eigen_values.real)
    entry_eigen_vectors = matrix_to_dictionary("eigen_vectors", eigen_vectors.real)

    dump[entry_cov[0]] = entry_cov[1]
    dump[entry_mean[0]] = entry_mean[1]
    dump["data"] = entries_data
    dump[entry_eigen_values[0]] = entry_eigen_values[1]
    dump[entry_eigen_vectors[0]] = entry_eigen_vectors[1]

    stream = open(path, 'w')
    yaml.dump(dump, stream)

    # plot for visual feedback
    plot(samples)



mu = [5.0, 5.0, -math.pi / 4.0]
sigma = [[0.0025, 0.0, 0.0],
         [0.0, 0.0025, 0.0],
         [0.0, 0.0, 0.1218]]

create_distribution(mu, sigma, 1000, '/tmp/distribution.yaml')


