import sys
import matplotlib.pyplot as plt

def read_file(file_name):
    with open(file_name, 'r') as f:
        header = f.readline().rstrip().split(',')
        columns = len(header)
        data = [[] for i in range(columns)]

        line = f.readline()
        while line != "":
            tokens = line.split(',')
            for i in range(columns):
                data[i].append(float(tokens[i]))
            line = f.readline()

        return header, columns, data





def run(file_name):
    header, columns, data = read_file(file_name)

    fig, ax = plt.subplots()
    ax.set_xlabel(header[0])
    curves = []

    for i in range(1, columns):
        # data[i][-1] = 0.0
        # curves.append(ax.fill(data[0], data[i], linestyle='dashed', label=header[i], alpha=0.3))
        curves.append(ax.plot(data[0], data[i], linewidth=1, label=header[i]))

    plt.legend()
    plt.show()

    return



if len(sys.argv) < 2:
    print("plot_filter_sate_log.py <file>")
else:
    run(sys.argv[1])