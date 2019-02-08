from example_08 import (remove_cache,
                        example_08_2_one_to_many,
                        example_08_3_many_to_one,
                        example_08_4_test_cached_trajectories)


def test_cached_trajectories(remove_cache: bool = False):
    if remove_cache:
        remove_cache.remove_cache()
    tc_go_to = example_08_2_one_to_many.get_one_to_many_trajectory_cache()
    tc_come_back = example_08_3_many_to_one.get_many_to_one_trajectory_cache()
    example_08_4_test_cached_trajectories.test_cached_trajectories(
        tc_go_to, tc_come_back)


if __name__ == '__main__':
    test_cached_trajectories()
