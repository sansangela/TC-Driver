import os

import numpy as np
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback, EvalCallback
from stable_baselines3.common.vec_env import sync_envs_normalization

from . import metrics

class NormalizedCheckpointCallback(CheckpointCallback):
    """
    New callback that saves the statistics of the normalizer
    when the model is saved

    Should be deprecated as the normalisation is now internal to the environment
    """

    def _on_step(self) -> bool:
        super()._on_step()

        if self.n_calls % self.save_freq == 0:
            path = os.path.join(
                self.save_path,
                f"{self.name_prefix}_{self.num_timesteps}_steps_stats.pkl",
            )
            self.training_env.save(path)

        return True

class MetricEvalCallback(BaseCallback):
    """
    Callback that automatically evaluates the metrics and stores them to
    weights and biases.
    """

    def __init__(self, eval_env, eval_freq, wandb_run, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.eval_env = eval_env
        self.eval_freq = eval_freq
        self.wandb_run = wandb_run


    def _on_step(self) -> bool:

        super()._on_step()
        
        # log data
        if self.eval_freq > 0 and self.n_calls % self.eval_freq == 0:
            # evaluate metric in 
            wandb_log = metrics.evaluate_metrics(
                self.eval_env, 
                self.model, 
                max_tsteps=self.eval_env.env_method("get_ep_len")[0], 
                track_shrink_coeff=0.31*1.5,
                idx=self.n_calls//self.eval_freq
                )
        
            self.wandb_run.log(wandb_log)
            
        return True

class CurriculumVelCallback(EvalCallback):

    def _on_step(self) -> bool:

        ########################################
        ### Code taken from sb3 EvalCallback ### 
        ########################################

        if self.eval_freq > 0 and self.n_calls % self.eval_freq == 0:
            # Sync training and eval env if there is VecNormalize
            sync_envs_normalization(self.training_env, self.eval_env)

            # Reset success rate buffer
            self._is_success_buffer = []

            episode_rewards, episode_lengths = evaluate_policy(
                self.model,
                self.eval_env,
                n_eval_episodes=self.n_eval_episodes,
                render=self.render,
                deterministic=self.deterministic,
                return_episode_rewards=True,
                warn=self.warn,
                callback=self._log_success_callback,
            )

            if self.log_path is not None:
                self.evaluations_timesteps.append(self.num_timesteps)
                self.evaluations_results.append(episode_rewards)
                self.evaluations_length.append(episode_lengths)

                kwargs = {}
                # Save success log if present
                if len(self._is_success_buffer) > 0:
                    self.evaluations_successes.append(self._is_success_buffer)
                    kwargs = dict(successes=self.evaluations_successes)

                np.savez(
                    self.log_path,
                    timesteps=self.evaluations_timesteps,
                    results=self.evaluations_results,
                    ep_lengths=self.evaluations_length,
                    **kwargs,
                )

            mean_reward, std_reward = np.mean(episode_rewards), np.std(episode_rewards)
            mean_ep_length, std_ep_length = np.mean(episode_lengths), np.std(episode_lengths)
            self.last_mean_reward = mean_reward

            if self.verbose > 0:
                print(f"Eval num_timesteps={self.num_timesteps}, " f"episode_reward={mean_reward:.2f} +/- {std_reward:.2f}")
                print(f"Episode length: {mean_ep_length:.2f} +/- {std_ep_length:.2f}")
            # Add to current Logger
            self.logger.record("eval/mean_reward", float(mean_reward))
            self.logger.record("eval/mean_ep_length", mean_ep_length)

            if len(self._is_success_buffer) > 0:
                success_rate = np.mean(self._is_success_buffer)
                if self.verbose > 0:
                    print(f"Success rate: {100 * success_rate:.2f}%")
                self.logger.record("eval/success_rate", success_rate)

            # Dump log so the evaluation results are printed with the correct timestep
            self.logger.record("time/total timesteps", self.num_timesteps, exclude="tensorboard")
            self.logger.dump(self.num_timesteps)

            if mean_reward > self.best_mean_reward:
                if self.verbose > 0:
                    print("New best mean reward!")
                if self.best_model_save_path is not None:
                    self.model.save(os.path.join(self.best_model_save_path, "best_model"))
                self.best_mean_reward = mean_reward
                # Trigger callback if needed
                if self.callback is not None:
                    return self._on_event()

            ########################################
            ###             new code             ### 
            ########################################

            if mean_reward/mean_ep_length >= 0.5*self.training_env.params['v_max']:
                new_vel =  1.05*self.training_env.params['v_max'] # TODO hardcoded param for curriculum
                new_vel = np.clip(new_vel, 0.1, 20) # TODO hardcoded velocity bounds
                self.training_env.env_method("update_max_vel", new_vel)

        return True