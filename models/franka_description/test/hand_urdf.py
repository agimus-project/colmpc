from .urdf_test_case import PKG, UrdfTestCase


class TestHandURDF(UrdfTestCase):
    def test_generate_urdf_without_args_is_possible(self):
        self.xacro("common/hand.urdf.xacro")  # does not throw


if __name__ == "__main__":
    import rosunit

    rosunit.unitrun(PKG, "common/hand.urdf.xacro", TestHandURDF)
