from matplotlib import pyplot as plt 
import pickle


colors_map = {
    0 : 'b-',
    1 : 'g-',
    2 : 'y-',
    3 : 'c-',
    4 : 'm-',
    5 : 'k-'
}


def main(paths_to_dumps: str):
    paths = paths_to_dumps.split()

    if len(paths) > 6:
        raise Warning('Given more than 6 experiments.! Will do only first 6!')
        paths = paths[:6]

    dumps = []
    for path in paths:
        dumps.append(pickle.load(open(path, "rb")))

    globalF1s, localF1s_lines, nlis, nics_cats, precs_lines = zip(*dumps)
    localF1s, lines = zip(*localF1s_lines)
    nics, cats = zip(*nics_cats)
    precs, lines_ = zip(*precs_lines)

    # plot globalF1s
    plt.figure(0)
    plt.grid(True)
    plt.xlabel('Number of Learned Categories', fontsize=15)
    plt.ylabel('Global Classification Accuracy', fontsize=15)
    plt.title('Global F1 vs Learned Categories')
    for idx, g in enumerate(globalF1s):
        x = list(range(1, len(g)+1))
        plt.plot(x, g, colors_map[idx]+'-', label=f'exp_{idx+1}', linewidth=2)
        plt.plot(x, g, 'ro', linewidth=2)
    plt.legend()
    plt.show()

    # plot localF1s
    plt.figure(1)
    plt.grid(True)
    plt.xlabel('Number of Learned Categories', fontsize=15)
    plt.ylabel('Protocol Accuracy', fontsize=15)
    plt.title('Local F1 vs Learned Categories')
    for idx, l in enumerate(localF1s):
        x = list(range(1, len(l)+1))
        plt.plot(x, l, colors_map[idx]+'-', label=f'exp_{idx+1}', linewidth=2)
        plt.plot(x, l, 'ro', linewidth=2)
        plt.plot(x, [lines[idx]] * len(l), colors_map[idx], linewidth=2)
    plt.legend()
    plt.show()

    # plot NLIs
    plt.figure(2)
    plt.grid(True)
    plt.ylabel('Question / Correct Iterations', fontsize=15)
    plt.xlabel('Number of Learned Categories', fontsize=15)
    plt.title('Number of Learned Categories vs Iterations')
    for idx, nli in enumerate(nlis):
        x = list(range(1, len(nli)+1))
        plt.plot(x, nli, colors_map[idx]+'-', label=f'exp_{idx+1}', linewidth=2)
        plt.plot(x, nli, 'ro', linewidth=2)
    plt.legend()
    plt.show()

    # plot NICs
    fig, ax = plt.subplots()
    plt.grid(True)
    fig.autofmt_xdate()
    width = 0.8
    num_exps = len(cats)
    x = list(range(len(cats[0])))   # x = list(range(max(list(map(len, cats))))))
    for idx, nic in enumerate(nics):
        x_fixed = list(map(lambda n: n + width*idx/num_exps, x))
        ax.bar(x_fixed[:len(nic)], nic, width/num_exps, label=f'exp_{idx+1}')
    ax.set_ylabel('Number of Stored Instances', fontsize=15)
    ax.set_title('Number of Stored Instances vs Categories')
    ax.set_xticks(x)
    ax.set_xticklabels(cats[0])
    ax.legend()
    plt.show()

    # plot graphs (first 200)
    plt.figure(4)
    plt.grid(b=True, which='both', linestyle='--')
    plt.minorticks_on()
    plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
    plt.axis([0,200,0,1.2])
    plt.xlabel('Question / Correction Iterations', fontsize=15)
    plt.ylabel('Protocol Accuracy', fontsize=15)
    plt.title('Protocol Accuracy vs Iterations (First 200)')
    for idx, prec in enumerate(precs):
        x = list(range(1, len(prec)+1))
        plt.plot(x, prec, colors_map[idx]+'-', label=f'exp_{idx+1}')
        plt.plot(x, [lines_[idx]] * len(prec), colors_map[idx], linewidth=2)
    plt.legend()
    plt.show()

    # plot graphs (Full)
    plt.figure(5)
    plt.grid(b=True, which='both', linestyle='--')
    plt.minorticks_on()
    plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
    plt.xlabel('Question / Correction Iterations', fontsize=15)
    plt.ylabel('Protocol Accuracy', fontsize=15)
    plt.title('Protocol Accuracy vs Iterations (Full)')
    for idx, prec in enumerate(precs):
        x = list(range(1, len(prec)+1))
        plt.plot(x, prec, colors_map[idx]+'-', label=f'exp_{idx+1}')
        plt.plot(x, [lines_[idx]] * len(prec), colors_map[idx], linewidth=2)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--paths_to_dumps', help='paths to all plot dump directories, given as a string, seperated by space', type=str, default=None)
    
    kwargs = vars(parser.parse_args())
    main(**kwargs)