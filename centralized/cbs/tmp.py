import requests
import eventlet
import time
 
eventlet.monkey_patch()
 
time_limit = 3  #set timeout time 3s
start = time.time()
with eventlet.Timeout(time_limit,False):
    time.sleep(5)
#     r=requests.get("https://me.csdn.net/dcrmg", verify=False)
    print('error')
end = time.time()
print('use time: ', end - start)
print('over')