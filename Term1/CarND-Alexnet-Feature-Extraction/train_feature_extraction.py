import pickle
import tensorflow as tf
from sklearn.model_selection import train_test_split
from alexnet import AlexNet
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split

EPOCHS = 1
BATCH_SIZE = 128
nb_classes = 43

# TODO: Load traffic signs data.
train_traffic_sign_dataset = "train.p"
with open(train_traffic_sign_dataset,mode ='rb') as train_data:
	train = pickle.load(train_data)

# TODO: Split data into training and validation sets.
X_all = train['features']
Y_all = train['labels']
X_all, Y_all = shuffle(X_all,Y_all)
X_train, X_test, y_train, y_test = train_test_split(
	X_all, Y_all, test_size=0.33, random_state=42)

# TODO: Define placeholders and resize operation.
x = tf.placeholder(tf.float32, (None, 32, 32, 3))
y = tf.placeholder(tf.int32, (None))
resized = tf.image.resize_images(x, (227, 227))
one_hot_y = tf.one_hot(y, 43)

# TODO: pass placeholder as first argument to `AlexNet`.
fc7 = AlexNet(resized, feature_extract=True)
# NOTE: `tf.stop_gradient` prevents the gradient from flowing backwards
# past this point, keeping the weights before and up to `fc7` frozen.
# This also makes training faster, less work to do!
fc7 = tf.stop_gradient(fc7)

# TODO: Add the final layer for traffic sign classification.
shape = (fc7.get_shape().as_list()[-1], nb_classes)  # use this shape for the weight matrix
my_W = tf.Variable(tf.random_uniform(shape))
my_b = tf.Variable(tf.zeros(nb_classes))
probs = tf.nn.softmax(tf.matmul(fc7, my_W) + my_b)

# TODO: Define loss, training, accuracy operations.
# HINT: Look back at your traffic signs project solution, you may
# be able to reuse some the code.

rate = 0.0009

logits = probs
cross_entropy = tf.nn.softmax_cross_entropy_with_logits(labels=one_hot_y, logits=logits)
loss_operation = tf.reduce_mean(cross_entropy)
optimizer = tf.train.AdamOptimizer(learning_rate = rate)
training_operation = optimizer.minimize(loss_operation,var_list=[fc8W, fc8b])


correct_prediction = tf.equal(tf.argmax(logits, 1), tf.argmax(one_hot_y, 1))
accuracy_operation = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

# TODO: Train and evaluate the feature extraction model.
def evaluate(X_data, y_data):
    num_examples = len(X_data)
    total_accuracy = 0
    sess = tf.get_default_session()
    for offset in range(0, num_examples, BATCH_SIZE):
        batch_x, batch_y = X_data[offset:offset+BATCH_SIZE], y_data[offset:offset+BATCH_SIZE]
        accuracy = sess.run(accuracy_operation, feed_dict={x: batch_x, y: batch_y})
        total_accuracy += (accuracy * len(batch_x))
    return total_accuracy / num_examples

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())
    num_examples = len(X_train)
    
    print("Training...")
    print()

    for i in range(EPOCHS):
        #make sure to re-shuffle the data each time
        X_train, y_train = shuffle(X_train, y_train)
        for offset in range(0, num_examples, BATCH_SIZE):
            end = offset + BATCH_SIZE
            batch_x, batch_y = X_train[offset:end], y_train[offset:end]
            sess.run(training_operation, feed_dict={x: batch_x, y: batch_y})
        print('computing validation accuracy')    
        validation_accuracy = evaluate(X_test, y_test)
        print("EPOCH {} ...".format(i+1))
        print("Validation Accuracy = {:.3f}".format(validation_accuracy))
        print()