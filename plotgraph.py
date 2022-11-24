import matplotlib.pyplot as plt
import copy

def plt_action(height):
    # x-coordinates of left sides of bars 
    left = [-0.2,-0.1, 0, 0.1, 0.2]
    height1 = list(height.values())

    # heights of bars
   # height = [947,14954,267,5465,591]
    norm = [(float(i)/sum(height1))*100 for i in height1] 
    # labels for bars
    tick_label = ['-0.2', '-0.1', '0', '0.1', '0.2']
    
    # plotting a bar chart
    plt.bar(left, norm, tick_label = tick_label,
            width = 0.02, color = ['red', 'green'])
    
    # naming the x-axis
    plt.xlabel('Localization error values')
    # naming the y-axis
    plt.ylabel('Density')
    # plot title
    plt.title('Action Taken')
    plt.savefig('action_taken.png', bbox_inches='tight')  
    # function to show the plot
   # plt.show()