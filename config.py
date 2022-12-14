config_1 = {
    'relevant_segments': 0,
    'done_after_collision': True,
    'memory_size': 40,  # 2000
    'batch_size': 8,
    'n_epochs': 4,
    'lr': 0.00075,  # 0.00006
    'anneal_lr': True,
    'N_actions': 5,  # 160*160
    'state_size': (200, 200),   # this config parameter is not used!
    'n_games': 30000,     # 80000
    'gamma': 0.70, #0.99
    'epsilon': -1,  # if epsilon is -1 the categorical distribution approach is used for exploration
    'entropy_coef': 0.01,
    'value_loss_coef': 0.5,
    'model_arch': 'normal',
    'log_interval': 10,
    'log_name': 'adv_1',    # this should be prot_1 if we want to train the protagonist and adv_1 if we train/evaluate an adversary -> some hardcoded line in csv_logger and main
    #'map_path': './maps/map_y.png',
    #'map_path': './maps/FinalGridMapv2.png',
    'map_path': './maps/map_g.png',
    'geo_path': './maps/map_geo.png',
    'start': [62,74], 
    'goal':[109,125],
    'eval_ep': 5,
    #'map_path': '/home/abdul/MasterThesis/Commisioning/Yannik/Stephan_Master_IAS/maps/map_y.png'
    'prob_const': 200,
    'eval_ep': 6
}

config_2 = {
    'relevant_segments': 0,
    'done_after_collision': True,
    'memory_size': 4000,  # 320
    'batch_size': 8,
    'n_epochs': 4,
    'lr': 0.00075,  # 0.00006
    'anneal_lr': True,
    'N_actions': 5,  # 160*160
    'state_size': (200, 200),   # this config parameter is not used!
    'n_games': 120000,
    'gamma': 0.70, #0.99
    'epsilon': -1,  # if epsilon is -1 the categorical distribution approach is used for exploration
    'entropy_coef': 0.01,
    'value_loss_coef': 0.5,
    'model_arch': 'normal',
    'log_interval': 20,
    'log_name': 'adv_1',    # this should be prot_1 if we want to train the protagonist and adv_1 if we train/evaluate an adversary
    'map_path': './maps/map_y.png',
    'geo_path': './maps/map_geo.png',
    'start': [85,52], 
    'goal':[109,125],
    'eval_ep': 5,
    #'map_path': '/home/abdul/MasterThesis/Commisioning/Yannik/Stephan_Master_IAS/maps/map_y.png'
    'prob_const': 200,
    'eval_ep': 5
}

config_3 = {

}

config_4 = {

}

configs = [config_1, config_2, config_3, config_4]
