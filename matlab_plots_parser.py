import os
import pickle
import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from typing import List, Tuple

_graph = 'graph1.m'
_globalF1 = 'global_F1_vs_learned_category.m'
#_localF1 = 'local_F1_vs_learned_category.m'
_nli = 'number_of_learned_categories_vs_Iterations.m'
_nic = 'number_of_stored_instaces_per_category.m'


def plot_globalF1(filename: str) -> List[float]:
    with open(filename, "r+") as f:
        lines = f.readlines()
    
    plt.figure(0)
    plt.grid(b=True, which='both', linestyle='--')
    plt.minorticks_on()
    plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
    
    globalF1 = list(filter(lambda l: l.startswith('global'), lines))
    globalF1 = globalF1[0].split('[')[-1][:-3]
    globalF1 = list(map(eval, globalF1.split(',')))

    axis = list(filter(lambda l: l.startswith('axis'), lines))
    axis = axis[0].split('[')[-1][:-4]
    plt.axis(list(map(eval, axis.split(','))))

    plt.xlabel('Number of Learned Categories', fontsize=15)
    plt.ylabel('Global Classification Accuracy', fontsize=15)
    plt.title('Global F1 vs Learned Categories')

    x = list(range(1, len(globalF1)+1))
    plt.plot(x, globalF1, 'b--', linewidth=2)
    plt.plot(x, globalF1, 'ro', linewidth=2)
    plt.show()

    return globalF1


def plot_localF1(filename: str) -> Tuple[List[float], float]:
    with open(filename, "r+") as f:
        lines = f.readlines()

    plt.figure(1)
    plt.grid(b=True, which='both', linestyle='--')
    plt.minorticks_on()
    plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

    localF1 = list(filter(lambda l: l.startswith('local'), lines))
    localF1 = localF1[0].split('[')[-1][:-3]
    localF1 = list(map(eval, localF1.split(',')))
    x = list(range(1, len(localF1)+1))
    
    axis = list(filter(lambda l: l.startswith('axis'), lines))
    axis = axis[0].split('[')[-1][:-4]
    plt.axis(list(map(eval, axis.split(','))))

    txt = list(filter(lambda l: l.startswith('text'), lines))
    txt = txt[0].split('(')[1].split(' ')[0].split(',')
    plt.text(eval(txt[0]), eval(txt[1]), 'Threshold', fontsize=10, fontweight='bold')

    line = list(filter(lambda l: l.startswith('line'), lines))
    line = eval(line[0].split('[')[2].split(' ')[0])
    plt.plot(x, [line]*len(localF1), 'k-', linewidth=2)

    plt.xlabel('Number of Learned Categories', fontsize=15)
    plt.ylabel('Protocol Accuracy', fontsize=15)
    plt.title('Local F1 vs Learned Categories')

    plt.plot(x, localF1, 'b--', linewidth=2)
    plt.plot(x, localF1, 'ro', linewidth=2)
    plt.show()

    return localF1, line


def plot_NLI(filename: str) -> List[int]:
    with open(filename, "r+") as f:
        lines = f.readlines()
    
    plt.figure(2)
    plt.grid(b=True, which='both', linestyle='--')
    plt.minorticks_on()
    plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
    
    nli = list(filter(lambda l: l.startswith('NLI'), lines))
    nli = nli[0].split('[')[-1][:-3]
    nli = list(map(eval, nli.split(',')))

    plt.ylabel('Question / Correct Iterations', fontsize=15)
    plt.xlabel('Number of Learned Categories', fontsize=15)
    plt.title('Number of Learned Categories vs Iterations')

    x = list(range(1, len(nli)+1))
    plt.plot(x, nli, 'b--', linewidth=2)
    plt.plot(x, nli, 'ro', linewidth=2)
    plt.show()

    return nli


def plot_NIC(filename: str) -> Tuple[List[int], List[str]]:
    with open(filename, "r+") as f:
        lines = f.readlines()

    fig, ax = plt.subplots()
    fig.autofmt_xdate()
    plt.grid(b=True, which='both', linestyle='--', alpha=0.4)
    #plt.minorticks_on()
    #plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)
    
    nic = list(filter(lambda l: l.startswith('NIC'), lines))
    nic = nic[0].split('[')[-1][:-3]
    nic = list(map(eval, nic.split(',')))

    plt.ylabel('Number of Stored Instances', fontsize=15)
    plt.title('Number of Stored Instances vs Categories')

    cats = list(filter(lambda l: l.startswith('list'), lines))
    cats = cats[0].split('{')[-1][:-3].split(',')

    plt.bar(cats, nic)
    plt.show()

    return nic, cats


def plot_graph(filename: str, axis: bool) -> Tuple[List[float], float]:
    with open(filename, "r+") as f:
        lines = f.readlines()
    
    plt.figure(4)
    plt.grid(b=True, which='both', linestyle='--')
    plt.minorticks_on()
    plt.grid(b=True, which='minor', color='#999999', linestyle='-', alpha=0.2)

    precision = list(filter(lambda l: l.startswith('Precision'), lines))
    precision = precision[0].split('[')[-1][:-5]
    precision = list(map(eval, precision.split(',')))
    
    plt.xlabel('Question / Correction Iterations', fontsize=15)
    plt.ylabel('Protocol Accuracy', fontsize=15)
    title = '(first 200 iterations)' if axis else '(full experiment)'
    plt.title(f'Protocol Accuracy vs Iterations {title}')

    if axis:
        plt.axis([0,200,0,1.2])
        ymax = 1.2
    else:
        axis = list(filter(lambda l: l.startswith('axis'), lines))
        axis = axis[0].split('[')[-1][:-4]
        plt.axis(list(map(eval, axis.split(','))))
        ymax = eval(axis.split(',')[-1])

    # do vertical lines
    texts = list(filter(lambda l: l.startswith('text'), lines))
    ticks = list(filter(lambda l: l.startswith('line'), lines))
    thresh = eval(ticks[0].split('[')[2].split()[0])
    plt.plot([thresh]*len(precision), 'g--', linewidth=2)
    plt.text(len(precision), thresh+.005, 'Threshold', fontsize=12, fontweight='bold')
    for idx in range(1, len(ticks)):
        x_txt, y_txt, category = texts[idx].split('(')[1].split(',')[0:3]
        plt.text(eval(x_txt), eval(y_txt), category, fontsize=15, fontweight='bold')
        x_tick = ticks[idx].split('[')[1].split(',')[0]
        plt.axvline(x=eval(x_tick), ymax=precision[eval(x_tick)]/ymax, linestyle='--', color='#FF0000', linewidth=1)

    plt.plot(precision, 'b-', linewidth=2)
    plt.show()

    return precision, thresh


def online_plot_main(path_to_m_files: str, dump_plots_flag: int):
        #verify they are there
        path = os.path.join(path_to_m_files, _globalF1)
        if os.path.isfile(path):
            globalF1 = plot_globalF1(path)
        else:
            globalF1 = []
            raise Warning(f'File {_globalF1} missing.')

        '''        
        path = os.path.join(path_to_m_files, _localF1)
	if os.path.isfile(path):
            localF1, line = plot_localF1(path)
        else:
            localF1, line = [], 0.
            raise Warning(f'File {_localF1} missing')
        '''

        path = os.path.join(path_to_m_files, _nli)
        if os.path.isfile(path):
            nli = plot_NLI(path)
        else:
            nli = []
            raise Warning(f'File {_nli} missing')
        path = os.path.join(path_to_m_files, _nic)
        if os.path.isfile(path):
            nic, cats = plot_NIC(path)
        else:
            nic, cats = [], []
            raise Warning(f'File {_nic} missing')
        path = os.path.join(path_to_m_files, _graph)
        if os.path.isfile(path):
            precision, line_ = plot_graph(path, False)
            _, _ = plot_graph(path, True)
        else:
            precision, line_ = [], 0.
            raise Warning(f'File {_graph} missing')

        if dump_plots_flag:
            #dump = (globalF1, (localF1, line), nli, (nic, cats), (precision, line_))
            dump = (globalF1, nli, (nic, cats), (precision, line_))
            path = path_to_m_files + '/dump_' + str(dump_plots_flag) + '.p'
            pickle.dump(dump, open(path, "wb"))


############################## OFFLINE PART #########################################


def get_confusion_matrix_nparray(directory_path: str):
    file_path = os.path.join(directory_path, 'visualize_confusion_matrix.m')
    if not os.path.isfile(file_path):
        raise ValueError(f'File visualize_confusion_matrix.m missing')
    
    confusion_array = []
    skip_line = True
    with open(file_path, 'r') as f:
        for line in f.readlines():
            if '];' in line and not skip_line:  # end of the matrix
                return np.array(confusion_array)
            if 'confmat = [ ' in line: # start of the matrix
                line = line.replace('confmat = [ ', '')  # remove the initial characters
                skip_line = False
            if skip_line:
                continue
            line = line.replace(';', '') # remove the trailing ;
            confusion_array.append(list(map(lambda i: int(i.strip()), line.split(', '))))
    error_message = 'The {} of the matlab array was not found in {}'.format('start' if skip_line else 'end', file_path)
    raise IOError(error_message)


def plot_conf_matrix(conf_matrix_array, categories):
    # quantity to percentages
    conf_matrix_array_percentages = (conf_matrix_array / conf_matrix_array.sum(axis=1)) * 100.0
    for i in range (conf_matrix_array_percentages.shape[1]):
       conf_matrix_array_percentages[i,:] = (conf_matrix_array[i,:] / conf_matrix_array[i,:].sum())* 100.0

    # Create the labels by combining the quantities and percentages
    labels = (
        np.asarray(
            [
                "{:.1f}%\n {:d}".format(percentage, quantity) 
                    for percentage, quantity in zip(conf_matrix_array_percentages.flatten(), conf_matrix_array.flatten())
            ]
        )
    ).reshape(conf_matrix_array_percentages.shape)
    
    # Cast the percentages to a pandas df which will be used for the heatmap colors
    cm_df = pd.DataFrame(conf_matrix_array_percentages, index = categories, columns = categories)

    # Actual plotting
    plt.figure(figsize=(8.5, 7))
    sns.heatmap(cm_df, annot=labels, cmap='hot_r', fmt="", square=True) # plot the heatmap
    plt.title('Accuracy: {:.2f}%'.format((conf_matrix_array.trace() / conf_matrix_array.sum()) * 100.0))
    plt.ylabel('Predicted Category')
    plt.xlabel('Target Category')
    plt.show()


def offline_plot_main(path_to_m_files: str, dump_plots_flag: int):
    # load the information in .m file to a np_array
    conf_matrix_array = get_confusion_matrix_nparray(path_to_m_files)
    # categories are always in same order
    categories = ['Bottle', 'Bowl', 'Flask', 'Fork', 'Knife', 'Mug', 'Plate', 'Spoon', 'Teapot', 'Vase']
    # plot the np array using seaborns heatmap
    plot_conf_matrix(conf_matrix_array, categories)


################################ Main part #################################################


def main(offline: bool, online: bool, **kwargs):
    if not online and not offline:
        raise ValueError("Please specify either the --offline or --online argument.")
    if online:
        online_plot_main(**kwargs)
    elif offline:
        offline_plot_main(**kwargs)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--path_to_m_files', help='path to extracted MATLAB files', type=str, default=None)
    parser.add_argument('-d', '--dump_plots_flag', help='set (1-10) to save plots', type=int, default=0)
    parser.add_argument('--offline', dest='offline', action='store_true', help='Produce the offline plots(part 1)')
    parser.add_argument('--online', dest='online', action='store_true', help='Plot the online plots(part 2)')
    
    kwargs = vars(parser.parse_args())
    main(**kwargs)
