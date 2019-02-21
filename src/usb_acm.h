
int usb_acm_setup(struct device **dev);
void interrupt_handler(struct device *dev);
void write_data(struct device *dev, const char *buf, int len);
void read_and_echo_data(struct device *dev, int *bytes_read);
