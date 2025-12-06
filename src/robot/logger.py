import datetime

class Logger:
    def __init__(self):
        self.log_folder = "resources\\logs"
        
    def log_changes(self):
        # Format date and time to be readable
        timestamp = str(datetime.datetime.now())
        date = timestamp.split(' ')[0].split('-')
        for i in range(len(date)):
            if date[i].startswith('0'):
                date[i] = date[i][1:]
        date.reverse()
        date = '-'.join(date)
        time = str(timestamp.split(' ')[1]).split('.')[0]
        
        # Get log message
        msg = input("Enter log message: ")
        
        # Open/create log file
        with open(f'{self.log_folder}\\{date}.txt', 'a') as log_file:
            log_file.write(f'{time} - {msg}\n')

logger = Logger()
logger.log_changes()