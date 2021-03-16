class BridgeInterprocess:
    def __init__(self, ip_dict, ip_event, ip_cmd_queue):
        self.ip_dict = ip_dict
        self.ip_event = ip_event
        self.ip_cmd_queue = ip_cmd_queue

    def availability(self):
        return (
            (self.ip_dict is not None)
            and (self.ip_event is not None)
            and (self.ip_cmd_queue is not None)
        )

    def send_dict_data(self, key, value):
        can_do = self.availability()
        if can_do is True:
            self.ip_dict[key] = value
            self.ip_event.set()
        return can_do

    def get_cmd_queue_no_wait(self):
        return self.ip_cmd_queue.get_nowait()
