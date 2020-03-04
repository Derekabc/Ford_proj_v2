import numpy as np

logs_dir = '../Data/results/'
ob_ls = np.load(logs_dir + '{}'.format('ob_ls') + '.npy')
ob_ls = np.squeeze(ob_ls)
ob_mean = np.mean(ob_ls, axis=0)
ob_std = np.std(ob_ls, axis=0)
ob_max = np.max(ob_ls, axis=0)
ob_min = np.min(ob_ls, axis=0)
print("")

obs_mean = np.array([4.83868269e+01, 9.26671424e+01, 6.41770269e+02, -3.11372911e+02,
                     6.78844516e-02, 1.27067008e-02, 1.46767778e+02])
obs_std = np.array([9.87342636e+00, 1.13743122e+02, 1.30954515e+02, 3.87827102e+02,
                    5.11422102e-02, 6.47789938e-02, 2.07343922e+02])

ob_test = ob_ls[110:120, :]
obs_normalized = (ob_test - obs_mean) / obs_std
