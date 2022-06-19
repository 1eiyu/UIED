from cnn.CNN import CNN
from cnn.Data import Data
test_data = Data()
test_data.load_data()
test_data.generate_training_data()
test_model = CNN('Elements', is_load=False)
test_model.train(data=test_data, epoch_num=8)
