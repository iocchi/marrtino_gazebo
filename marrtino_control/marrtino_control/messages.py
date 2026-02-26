import threading
import queue


class MessagePublisher:

    def __init__(self, module, topic, Q):
        self.module = module
        self.topic = topic
        self.Q = Q

    def publish(self, message):
        for k in self.Q.keys():
            if k[1] == self.topic:
                self.Q[k].put(message)

class MessageSubscriber:

    def __init__(self, q: queue.Queue):
        self.q = q

    def receive(self, timeout=None):
        return self.q.get(timeout=timeout)  # blocking if queue is empty


class MessageDispatcher:

    def __init__(self):
        self.Q = {}  # dictionary of message queues, key: (module, topic)

    def subscriber(self, module, topic):
        # create a new queue
        if (module,topic) not in self.Q.keys():
            self.Q[(module,topic)] = queue.Queue()
            print(f"MD -- Subscriber {module},{topic}")
        return MessageSubscriber(self.Q[(module,topic)])

    def publisher(self, module, topic):
        print(f"MD -- Publisher {module},{topic}")
        return MessagePublisher(module, topic, self.Q)


md = None

def getMessageDispatcher():
    global md
    if md is None:
        md = MessageDispatcher()
    return md


'''
General use with multi-thread

def p1(md, ...):
    sub1 = md.subscriber("p1", "t1")
    sub2 = md.subscriber("p1", "t2")
    pub1 = md.publisher("p1", "t3")
    pub2 = md.publisher("p1", "t4")

    while ...:
        # read inputs (wait for available messages)
        i1 = sub1.receive() # blocking
        i2 = sub2.receive() # blocking

        # processing
        ....

        # send outputs
        pub1.publish(o1)
        pub2.publish(o2)

    

def p2(md, ...):
    ...

def p3(md, ...):
    ...


def main():

    md = MessageDispatcher()

    p1_t = threading.Thread(target=p1, args=(md, ...))
    p2_t = threading.Thread(target=p2, args=(md, ...))
    p3_t = threading.Thread(target=p3, args=(md, ...))

    p1_t.start()
    p2_t.start()
    p3_t.start()

    p1_t.join()
    p2_t.join()
    p3_t.join()

'''

