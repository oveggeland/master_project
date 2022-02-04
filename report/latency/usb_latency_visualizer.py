import rosbag
import numpy as np
import matplotlib.pyplot as plt


if __name__ == "__main__":
    bagfile = "offset_test.bag"
    topics = ["/imu_acc_ar", "/cam0/cam0"]
    exp_diff = 5	

    start_point = 10
    end_point = 30

    with rosbag.Bag(bagfile, 'r') as bag:
        max_counts = max([bag.get_message_count(tpc) for tpc in topics])
        data = np.zeros((len(topics), max_counts))
        prev = 0

        for j, tpc in enumerate(topics):
            for i, (topic, msg, ts) in enumerate(bag.read_messages(topics=tpc)):
                if i == 0:
                    prev = msg.header.stamp.to_sec()
                else:
                    temp = msg.header.stamp.to_sec()
                    diff = temp-prev
                    prev = temp
                    print(prev)
                    data[j, i] = diff/1000000 # to milliseconds

        for i, tpc in enumerate(topics):
            #data = diffs[i, start_point:end_point]
            print(data[i, :100])
            plt.plot(data[i], label=f"Measured difference", marker='o')
            plt.ylim(top=data.max()*1.05)


        plt.plot(np.ones(data.shape[1])*exp_diff, color='r', label='Expected difference')
        plt.ylabel("Time difference [ms]")
        plt.xlabel("Message number")
        #plt.xticks(range(start_point, data.shape[0]+1, data.shape[0]//10))
        plt.legend(loc='upper right')
        plt.title(f"Time difference between measurements of {topics[0]}")
        plt.xlim(left=start_point)
        if end_point != -1:
            plt.xlim(right=end_point)

        name = bagfile.split(".")[0] + "_stamps.svg"
        plt.savefig(name)
        plt.show()


        # Overall consistency check!
        print("Average differnece is", np.average(data))



