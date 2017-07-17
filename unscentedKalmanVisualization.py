import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import argparse

#///\ unscentedKF class
#///\ function __init__(self,args)
#///\ parse(self)
#///\ nisVisual(self)
#///\ rmseVisual(self)
class unscentedKF:
    def __init__(self,args):
        self.data_1_dir = args.data_1_dir
        self.data_2_dir = args.data_2_dir
        self.data_synthetic_dir = args.data_synthetic_dir
        self.rmse_data_1_dir = args.rmse_data_1_dir
        self.rmse_data_2_dir = args.rmse_data_2_dir
        self.rmse_synthetic_dir = args.rmse_synthetic_dir
        self.df1 = []
        self.df2 = []
        self.df3 = []
        self.df_rmse1 = []
        self.df_rmse2 = []
        self.df_rmse_synthetic = []
    #///\ function: parse(self)
    #///\ details: parse data and assign to lists
    def parse(self):
        self.df1 = pd.read_table(self.data_1_dir,sep='\t')
        self.df2 = pd.read_table(self.data_2_dir,sep='\t')
        self.df_synthetic = pd.read_table(self.data_synthetic_dir,sep='\t')
        self.df_rmse1 = pd.read_table(self.rmse_data_1_dir,sep='\t')
        self.df_rmse2 = pd.read_table(self.rmse_data_2_dir,sep='\t')
        self.df_rmse_synthetic = pd.read_table(self.rmse_synthetic_dir,sep='\t')
    #///\ function: ninVisual(self)
    #///\ details: plot NIS comparing with Radar and Laser data
    def nisVisual(self):
        fig = plt.figure(figsize=(14, 5))
        #fig.suptitle('NIS', fontsize=16, fontweight='bold')
        
        ax = fig.add_subplot(121)
        
        ax.set_title('Dataset 1', fontsize=14, fontweight='bold')
        ax.set_xlabel('Time')
        ax.set_ylabel('NIS Radar')
        
        ax.plot(self.df1['NIS'])
        ax.plot((0, 1400), (7.8, 7.8), 'r-', linewidth=2, label = '7.8: Radar 95%')
        ax.legend(loc='upper right')
        ax.axis([0, 1230 , 0, 45])
        
        ax = fig.add_subplot(122)
        
        ax.set_title('Dataset 2', fontsize=14, fontweight='bold')
        ax.set_xlabel('Time')
        ax.set_ylabel('NIS Laser')
        
        ax.plot(self.df2['NIS'])
        ax.plot((0, 200), (5.99, 5.99), 'g.-', linewidth=2, label = '5.99: Laser 95%')
        ax.legend(loc='upper right')
        ax.axis([0, 210 , 0, 10])
              
        #plt.show()
        plt.savefig('./nis_Vis.png')
    #///\ function: rmseVisual(self)
    #///\ details: plot estimation and ground truth comparison
    def rmseVisual(self):
        fig = plt.figure(figsize=(16,8))
        fig.suptitle('Unscented Kalman Filter - Results', fontsize=16, fontweight='bold')
        
        ### Dataset 1 -----------------------------
        ax = fig.add_subplot(131)
        
        ax.set_title('Dataset 1', fontsize=14, fontweight='bold')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        
        # show px, py and true values
        ax.scatter(self.df1["px_ground_truth"], self.df1["py_ground_truth"], alpha=0.8, c='r', label="Ground Truth")
        ax.scatter(self.df1["px_state"], self.df1["py_state"], alpha=0.8, marker='x',c='g', label="UKF estimation")
        ax.legend(loc='upper left')

        # show RMSE1
        rms_txt1 = 'RMSE\n' +\
                   'px: ' + str(self.df_rmse1['px'][0]) + '\n' +\
                   'py: ' + str(self.df_rmse1['py'][0]) + '\n' +\
                   'vx: ' + str(self.df_rmse1['vx'][0]) + '\n' +\
                   'vy: ' + str(self.df_rmse1['vy'][0]) + '\n' 

        ax.text(4.2, -4, rms_txt1, style='italic', bbox={'alpha': 0.5, 'pad': 5})
        #"""
        
        ### Dataset 2 -----------------------------
        ax = fig.add_subplot(132)
        
        ax.set_title('Dataset 2', fontsize=14, fontweight='bold')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        
        # show px, py and true values
        ax.scatter(self.df2["px_ground_truth"], self.df2["py_ground_truth"], alpha=0.8, c='r', label="Ground Truth")
        ax.scatter(self.df2["px_state"], self.df2["py_state"], alpha=0.8, marker='x',c='g', label="UKF estimation")
        ax.legend(loc='upper left')
        
        # show RMSE2 
        rms_txt2 = 'RMSE\n' +\
                   'px: ' + str(self.df_rmse2['px'][0]) + '\n' +\
                   'py: ' + str(self.df_rmse2['py'][0]) + '\n' +\
                   'vx: ' + str(self.df_rmse2['vx'][0]) + '\n' +\
                   'vy: ' + str(self.df_rmse2['vy'][0]) + '\n' 

        ax.text(0, 27, rms_txt2, style='italic', bbox={'alpha': 0.5, 'pad': 5})
        
        ### Synthetic -----------------------------
        ax = fig.add_subplot(133)
        
        ax.set_title('Synthetic', fontsize=14, fontweight='bold')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        
        # show px, py and true values
        ax.scatter(self.df_synthetic["px_ground_truth"], self.df_synthetic["py_ground_truth"], alpha=0.8, c='r', label="Ground Truth")
        ax.scatter(self.df_synthetic["px_state"], self.df_synthetic["py_state"], alpha=0.8, marker='x',c='g', label="UKF estimation")
        ax.legend(loc='upper left')
        
        # show RMSE2 
        rms_txt3 = 'RMSE\n' +\
                   'px: ' + str(self.df_rmse_synthetic['px'][0]) + '\n' +\
                   'py: ' + str(self.df_rmse_synthetic['py'][0]) + '\n' +\
                   'vx: ' + str(self.df_rmse_synthetic['vx'][0]) + '\n' +\
                   'vy: ' + str(self.df_rmse_synthetic['vy'][0]) + '\n' 

        ax.text(-27, 13, rms_txt3, style='italic', bbox={'alpha': 0.5, 'pad': 5})
        
        #plt.show()
        plt.savefig('./estimation_Vs_groundtruth_Vis.png')


def main():             
    parser = argparse.ArgumentParser(description="Unscented Kalma Filter Term2/Project2")
    parser.add_argument('-d1', help='data 1 directory', dest='data_1_dir',type=str,default='./output/laser_radar_data-1.txt')
    parser.add_argument('-d2', help='data 2 directory', dest='data_2_dir',type=str,default='./output/laser_radar_data-2.txt')
    parser.add_argument('-dsyn', help='data synthetic directory', dest='data_synthetic_dir',type=str,default='./output/laser_radar_out_synthetic.txt')
    parser.add_argument('-d1_rmse', help='rmse data 1 directory', dest='rmse_data_1_dir',type=str,default='./output/rmse_out_data1.txt')
    parser.add_argument('-d2_rmse', help='rmse data 1 directory', dest='rmse_data_2_dir',type=str,default='./output/rmse_out_data2.txt')
    parser.add_argument('-dsyn_rmse', help='rmse syntheric data directory', dest='rmse_synthetic_dir',type=str,default='./output/rmse_out_synthetic.txt')

    args = parser.parse_args()
    #///\ unscentedKF class instantiation
    ukf = unscentedKF(args)
    ukf.parse()
    ukf.nisVisual()
    ukf.rmseVisual()

if __name__ == '__main__':
    main()