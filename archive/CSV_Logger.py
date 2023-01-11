# if i still need to write a nested list in row: https://stackoverflow.com/questions/32855996/python-save-arbitrarily-nested-list-to-csv

import csv
import sys

import numpy as np

logs_path = "logs/"


class CSV_Logger:
    """
    A logger which writes data into CSV files
    """

    def __init__(
        self,
        N_epochs,
        N_games,
        memory_size,
        batch_size,
        gamma,
        lr,
        anneal_lr,
        epsilon,
        entropy_coef,
        value_loss_coef,
        relevant_segments,
        done_after_collision,
        model_arch,
        test_mode,
        path,
    ):
        self.N_epochs = N_epochs
        """Number of epochs"""
        self.N_games = N_games
        """Number of games which were run"""
        self.memory_size = memory_size
        """Size of the PPO memory"""
        self.batch_size = batch_size
        """Size of the batch"""
        self.gamma = gamma
        """Discount factor. Defaults to 0.99"""
        self.lr = lr
        """Learning rate"""
        self.anneal_lr = anneal_lr
        self.epsilon = epsilon
        """Defines clipping interval. Defaults to 0.5"""
        self.entropy_coef = entropy_coef
        """Regularizer.  Helps prevent premature convergence of one action probability dominating\
             the policy and preventing exploration. Defaults to 0.01"""
        self.value_loss_coef = value_loss_coef
        """"Loss coefficient of neural network. Defaults to 0.5"""
        self.relevant_segments = relevant_segments
        """How many segments have been considered in the state for the sim-gap-adv"""
        self.done_after_collision = done_after_collision
        """If the MAARL session terminates after a collision occured"""
        self.model_arch = model_arch
        """Just for faster testing of different hyperparameters. usually model_arch will just be "normal" """
        self.test_mode = test_mode
        """If MAARL is run in training mode (False) or testing mode (True)"""
        self.path = path
        """Path where the CSV file should be written to"""
        if self.test_mode:
            path_extension = "_test_log.csv"
        else:
            path_extension = "_train_log.csv"
        self.path = self.path + path_extension
        self.init_logger()

    def init_logger(self):
        """
        Writes hyperparameters to CSV file and header for the episode logs
        """
        with open(logs_path + self.path, "w", newline="") as csvfile:
            writer = csv.writer(csvfile, delimiter=";")
            writer.writerow(
                ["N_Epochs"]
                + ["N_games"]
                + ["memory_size"]
                + ["batch_size"]
                + ["gamma"]
                + ["lr"]
                + ["anneal_lr"]
                + ["epsilon"]
                + ["entropy_coef"]
                + ["value_loss_coef"]
                + ["relevant_segments"]
                + ["done_after_collision"]
                + ["model_arch"]
            )
            writer.writerow(
                [
                    self.N_epochs,
                    self.N_games,
                    self.memory_size,
                    self.batch_size,
                    self.gamma,
                    self.lr,
                    self.anneal_lr,
                    self.epsilon,
                    self.entropy_coef,
                    self.value_loss_coef,
                    self.relevant_segments,
                    self.done_after_collision,
                    self.model_arch,
                ]
            )
            writer.writerow("")
            writer.writerow(
                (
                    ["step"]
                    + ["episode"]
                    + ["value"]
                    + ["action[-10°]"]
                    + ["action[-5°]"]
                    + ["action[0°]"]
                    + ["action[5°]"]
                    + ["action[10°]"]
                    + ["reward"]
                    + ["collision_status"]
                    + ["length_ratio"]
                )
            )

    def add_rows(
        self, episodes, probs, values, rewards, collision_status, length_ratio
    ):
        """
        Add a rows with the episode data of the adversary actions. All values are given as a list => lists must be equally long

        Attributes:
            episodes (list (int)): Number of episodes
            probs (list (list(float))): All probabilities of the actions in the action space  to be taken
            values (list (float)): risk values
            rewards (list (float)): The rewards of the step
            collision_status (either 0 or 1): If collision happend
            length_ratio (): Always 0
        """
        with open(logs_path + self.path, "a", newline="") as csvfile:
            writer = csv.writer(csvfile, delimiter=";")
            for episode in range(0, len(episodes)):
                for step in range(0, len(episodes[episode])):
                    if (self.path == "adv_1_train_log.csv") or (
                        self.path == "adv_1_test_log.csv"
                    ):
                        # TODO: hardcoded for now! has to be changed, so other model names in config are accepted!
                        writer.writerow(
                            [
                                step + 1,
                                episodes[episode][step],
                                values[episode][step],
                                probs[episode][step][0],
                                probs[episode][step][1],
                                probs[episode][step][2],
                                probs[episode][step][3],
                                probs[episode][step][4],
                                rewards[episode][step],
                                collision_status[episode][step],
                                0,
                            ]
                        )
                    else:  # for prot we have episodic RL -> one step only per episode, also we use the length_ratio thats why we differentiate here...
                        writer.writerow(
                            [
                                step + 1,
                                episodes[episode][step],
                                values[episode][step],
                                probs[episode][step][0],
                                probs[episode][step][1],
                                probs[episode][step][2],
                                probs[episode][step][3],
                                probs[episode][step][4],
                                rewards[episode][step],
                                collision_status[episode],
                                length_ratio[episode],
                            ]
                        )


class CSV_Reader:
    """
    A class which reads existing CSV data
    """

    def __init__(self, path, test_mode=True):
        self.path = path
        """Path of the CSV file"""
        self.test_mode = test_mode
        """If MAARL is run in training mode (False) or testing mode (True)"""
        if test_mode:
            path_extension = "_test_log.csv"
        else:
            path_extension = "_train_log.csv"
        self.path = self.path + path_extension
        self.data_labels = []
        """Labels of the columns of the CSV"""
        self.steps = []
        """Number of step"""
        self.actions_0 = []
        """action[-10°]"""
        self.actions_1 = []
        """action[-5°]"""
        self.actions_2 = []
        """action[0°]"""
        self.actions_3 = []
        """action[5°]"""
        self.actions_4 = []
        """action[10°]"""
        self.values = []
        self.rewards = []
        """reward of the data"""
        self.collisions = []
        """If collision occured"""
        self.length_ratios = []
        """Lenght ratio"""
        self.read_data()

    def read_data(self):
        self.data_labels = [
            "step",
            "episode",
            "value",
            "action[-10°]",
            "action[-5°]",
            "action[0°]",
            "action[5°]",
            "action[10°]",
            "reward",
            "collision",
            "length_ratio",
        ]

        with open((logs_path + self.path), newline="") as csvfile:
            reader = csv.reader(csvfile, delimiter=";")
            # row_count = sum(1 for row in reader)
            # print("Rows: ", row_count)
            i = 0
            step_index = self.data_labels.index("step")
            reward_index = self.data_labels.index("reward")
            action_0_index = self.data_labels.index("action[-10°]")
            value_index = self.data_labels.index("value")
            collision_index = self.data_labels.index("collision")
            length_ratio_index = self.data_labels.index("length_ratio")

            for row in reader:
                i += 1
                if i < 5:
                    continue
                # if i % 2:   # only take every second data point
                #     continue
                # print(int(row[step_index]))
                self.steps.append(int(row[step_index]))
                self.rewards.append(float(row[reward_index]))
                self.actions_0.append(np.float(row[action_0_index]))
                self.actions_1.append(np.float(row[action_0_index + 1]))
                self.actions_2.append(np.float(row[action_0_index + 2]))
                self.actions_3.append(np.float(row[action_0_index + 3]))
                self.actions_4.append(np.float(row[action_0_index + 4]))
                self.values.append(np.float(row[value_index]))
                self.collisions.append(np.float(row[collision_index]))
                self.length_ratios.append(np.float(row[length_ratio_index]))
                # if i == 8512:
                #     break

            avg = np.mean(self.rewards)
            var = np.var(self.rewards)
            print("read data.")
            print("mean reward:", np.mean(self.rewards))
            print("variance:", np.var(self.rewards))

            print("mean collision", np.mean(self.collisions))
            print("mean action 0:", np.mean(self.actions_0))
            print("mean action 1:", np.mean(self.actions_1))

            if self.test_mode:
                with open((logs_path + self.path), "a", newline="") as csvfile:
                    writer = csv.writer(csvfile, delimiter=";")
                    writer.writerow("")
                    writer.writerow((["avg"] + ["var"]))
                    writer.writerow([avg, var])
