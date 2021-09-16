import requests
from bs4 import BeautifulSoup
import traceback
from collections import namedtuple
import numpy as np
from datetime import datetime
import re
import time



import time

JointTarget=namedtuple('JointTarget', ['robax', 'extax'])
RobTarget=namedtuple('RobTarget', ['trans','rot','robconf','extax'])

class RAPID(object):

    def __init__(self, base_url='http://127.0.0.1:80', username='Default User', password='robotics'):
        self.base_url=base_url
        self.auth=requests.auth.HTTPDigestAuth(username, password)
        self._session=requests.Session()
        self._rmmp_session=None
        self._rmmp_session_t=None
        
    def _do_get(self, relative_url):
        url="/".join([self.base_url, relative_url])
        res=self._session.get(url, auth=self.auth)
        try:            
            return self._process_response(res)
        finally:
            res.close()
    

    def _do_post(self, relative_url, payload=None):
        url="/".join([self.base_url, relative_url])
        res=self._session.post(url, data=payload, auth=self.auth)
        try:
            return self._process_response(res)
        finally:
            res.close()

    def _process_response(self, response):        
        soup=BeautifulSoup(response.text)

        if (response.status_code == 500):
            raise Exception("Robot returning 500 Internal Server Error")
    
        if (response.status_code == 200 or response.status_code == 201  \
            or response.status_code==202 or response.status_code==204):
            
            return soup.body
        
        if soup.body is None:
            raise Exception("Robot returning HTTP error " + str(response.status_code))
        
        error_code=int(soup.find('span', attrs={'class':'code'}).text)
        error_message1=soup.find('span', attrs={'class': 'msg'})
        if (error_message1 is not None):
            error_message=error_message1.text
        else:
            error_message="Received error from ABB robot: " + str(error_code)

        raise ABBException(error_message, error_code)

    def start(self, cycle='asis'):
        payload={"regain": "continue", "execmode": "continue" , "cycle": cycle, "condition": "none", "stopatbp": "disabled", "alltaskbytsp": "false"}
        res=self._do_post("rw/rapid/execution?action=start", payload)

    def stop(self):
        payload={"stopmode": "stop"}
        res=self._do_post("rw/rapid/execution?action=stop", payload)

    def resetpp(self):
        res=self._do_post("rw/rapid/execution?action=resetpp")

    def get_execution_state(self):
        soup = self._do_get("rw/rapid/execution")
        ctrlexecstate=soup.find('span', attrs={'class': 'ctrlexecstate'}).text
        cycle=soup.find('span', attrs={'class': 'cycle'}).text
        return RAPIDExecutionState(ctrlexecstate, cycle)
    
    def get_controller_state(self):
        soup = self._do_get("rw/panel/ctrlstate")
        return soup.find('span', attrs={'class': 'ctrlstate'}).text
    
    def get_operation_mode(self):
        soup = self._do_get("rw/panel/opmode")        
        return soup.find('span', attrs={'class': 'opmode'}).text
    
    def get_digital_io(self, signal, network='Local', unit='DRV_1'):
        soup = self._do_get("rw/iosystem/signals/" + network + "/" + unit + "/" + signal)        
        state = soup.find('span', attrs={'class': 'lvalue'}).text
        return int(state)
    
    def set_digital_io(self, signal, value, network='Local', unit='DRV_1'):
        lvalue = '1' if bool(value) else '0'
        payload={'lvalue': lvalue}
        res=self._do_post("rw/iosystem/signals/" + network + "/" + unit + "/" + signal + "?action=set", payload)
    
    def get_rapid_variable(self, var):
        soup = self._do_get("rw/rapid/symbol/data/RAPID/T_ROB1/" + var)        
        state = soup.find('span', attrs={'class': 'value'}).text
        return state
    
    def set_rapid_variable(self, var, value):
        payload={'value': value}
        res=self._do_post("rw/rapid/symbol/data/RAPID/T_ROB1/" + var + "?action=set", payload)
        
    def read_event_log(self, elog=0):
        o=[]
        soup = self._do_get("rw/elog/" + str(elog) + "/?lang=en")
        state=soup.find('div', attrs={'class': 'state'})
        ul=state.find('ul')
        
        for li in ul.findAll('li'):
            def find_val(v):
                return li.find('span', attrs={'class': v}).text
            msg_type=int(find_val('msgtype'))
            code=int(find_val('code'))
            tstamp=datetime.strptime(find_val('tstamp'), '%Y-%m-%d T  %H:%M:%S')
            title=find_val('title')
            desc=find_val('desc')
            conseqs=find_val('conseqs')
            causes=find_val('causes')
            actions=find_val('actions')
            args=[]
            nargs=int(find_val('argc'))
            for i in xrange(nargs):
                arg=find_val('arg%d' % (i+1))
                args.append(arg)
            
            o.append(RAPIDEventLogEntry(msg_type,code,tstamp,args,title,desc,conseqs,causes,actions))
        return o
    
    def get_jointtarget(self, mechunit="ROB_1"):
        soup=self._do_get("rw/motionsystem/mechunits/" + mechunit + "/jointtarget")
        state=str(soup.find('li', attrs={'class': 'ms-jointtarget'}))
        robjoint=np.zeros((6,))
        i=0
        for match in re.finditer('class=\"rax_(\\d)">(-?\\d+(?:\\.\\d+)?)',state):
            j=int(match.groups()[0])
            assert i+1==j
            a=float(match.groups()[1])
            robjoint[j-1]=np.deg2rad(a)
            i+=1
            
        #robjoint=np.array([np.deg2rad(float(state.find('span', attrs={'class': 'rax_' + str(i+1)}).text)) for i in xrange(6)]) 
        #extjoint=np.array([np.deg2rad(float(state.find('span', attrs={'class': 'eax_' + chr(i)}).text)) for i in xrange(ord('a'),ord('g'))])
        extjoint=None
        return JointTarget(robjoint,extjoint)
        
    def get_robtarget(self, mechunit='ROB_1', tool='tool0', wobj='wobj0', coordinate='Base'):
        soup=self._do_get("rw/motionsystem/mechunits/" + mechunit + "/robtarget?tool=%s&wobj=%s&coordinate=%s" % (tool, wobj, coordinate))
        state=soup.find('li', attrs={'class': 'ms-robtargets'})
        trans=np.array([(float(state.find('span', attrs={'class': i}).text)/1000.0) for i in 'xyz'])
        rot=np.array([(float(state.find('span', attrs={'class': 'q' + str(i+1)}).text)) for i in xrange(4)])
        robconf=np.array([(float(state.find('span', attrs={'class': i}).text)) for i in ['cf1','cf4','cf6','cfx']])
        extax=np.array([np.deg2rad(float(state.find('span', attrs={'class': 'eax_' + chr(i)}).text)) for i in xrange(ord('a'),ord('g'))])
        return RobTarget(trans,rot,robconf,extax)
    
    def _rws_value_to_jointtarget(self, val):
        v1=re.match('^\\[\\[([^\\]]+)\\],\\[([^\\]]+)\\]',val)
        robax = np.deg2rad(np.fromstring(v1.groups()[0],sep=','))
        extax = np.deg2rad(np.fromstring(v1.groups()[1],sep=','))
        return JointTarget(robax,extax)
    
    def _jointtarget_to_rws_value(self, val):
        assert np.shape(val[0]) == (6,)
        assert np.shape(val[1]) == (6,)
        robax=','.join([format(x, '.4f') for x in np.rad2deg(val[0])])
        extax=','.join([format(x, '.4f') for x in np.rad2deg(val[1])])
        rws_value="[[" + robax + "],[" + extax + "]]"
        return rws_value
    
    def get_rapid_variable_jointtarget(self, var):
        v = self.get_rapid_variable(var)
        return self._rws_value_to_jointtarget(v)
    
    def set_rapid_variable_jointtarget(self,var,value):
        rws_value=self._jointtarget_to_rws_value(value)
        self.set_rapid_variable(var, rws_value)
            
    def _rws_value_to_jointtarget_array(self,val):
        m1=re.match('^\\[(.*)\\]$',val)
        if len(m1.groups()[0])==0:
            return []
        arr=[]
        val1=m1.groups()[0]
        while len(val1) > 0:
            m2=re.match('^(\\[\\[[^\\]]+\\],\\[[^\\]]+\\]\\]),?(.*)$',val1)            
            val1 = m2.groups()[1]
            arr.append(self._rws_value_to_jointtarget(m2.groups()[0]))
        
        return arr       
    
    def _jointtarget_array_to_rws_value(self, val):
        return "[" + ','.join([self._jointtarget_to_rws_value(v) for v in val]) + "]"
    
    def get_rapid_variable_jointtarget_array(self, var):
        v = self.get_rapid_variable(var)
        return self._rws_value_to_jointtarget_array(v)
    
    def set_rapid_variable_jointtarget_array(self,var,value):
        rws_value=self._jointtarget_array_to_rws_value(value)
        self.set_rapid_variable(var, rws_value)

    def get_rapid_variable_num(self, var):
        return float(self.get_rapid_variable(var))
    
    def set_rapid_variable_num(self, var, val):
        self.set_rapid_variable(var, str(val))
        
    def get_rapid_variable_num_array(self, var):
        val1=self.get_rapid_variable(var)
        m=re.match("^\\[([^\\]]*)\\]$", val1)
        val2=m.groups()[0].strip()
        return np.fromstring(val2,sep=',')
    
    def set_rapid_variable_num_array(self, var, val):
        self.set_rapid_variable(var, "[" + ','.join([str(s) for s in val]) + "]")
    
    
    def read_ipc_message(self, queue_name, timeout=0):
        
        o=[]
        
        timeout_str=""
        if timeout > 0:
            timeout_str="&timeout=" + str(timeout)
        
        soup=self._do_get("rw/dipc/" + queue_name + "/?action=dipc-read" + timeout_str)
                            
        state=soup.find('div', attrs={'class': 'state'})
        ul=state.find('ul')
        
        for li in ul.findAll('li'):
            def find_val(v):
                return li.find('span', attrs={'class': v}).text
            msgtype=find_val('dipc-msgtype')
            cmd=int(find_val('dipc-cmd'))
            userdef=int(find_val('dipc-userdef'))
            data=find_val('dipc-data')
            
            o.append(RAPIDIpcMessage(data,userdef,msgtype,cmd))
            
            #o.append(RAPIDEventLogEntry(msg_type,code,tstamp,args,title,desc,conseqs,causes,actions))
        return o
    
    def send_ipc_message(self, target_queue, data, queue_name="rpi_abb_irc5", cmd=111, userdef=1, msgtype=1 ):
        payload={"dipc-src-queue-name": queue_name, "dipc-cmd": str(cmd), "dipc-userdef": str(userdef), \
                 "dipc-msgtype": str(msgtype), "dipc-data": data}
        res=self._do_post("rw/dipc/" + target_queue + "?action=dipc-send", payload)
    
    def get_ipc_queue(self, queue_name):
        res=self._do_get("rw/dipc/" + queue_name + "?action=dipc-read")
        return res
    
    def try_create_ipc_queue(self, queue_name, queue_size=4440, max_msg_size=444):
        try:
            payload={"dipc-queue-name": queue_name, "dipc-queue-size": str(queue_size), "dipc-max-msg-size": str(max_msg_size)}
            self._do_post("rw/dipc?action=dipc-create", payload)
            return True
        except ABBException as e:
            if e.code==-1073445879:
                return False
            raise
    
    def request_rmmp(self, timeout=5):
        t1=time.time()
        self._do_post('users/rmmp', {'privilege': 'modify'})
        while time.time() - t1 < timeout:
            
            soup=self._do_get('users/rmmp/poll')
            status=soup.find('span', {'class': 'status'}).text
            if status=="GRANTED":
                self.poll_rmmp()
                return
            elif status!="PENDING":
                raise Exception("User did not grant remote access")                               
            time.sleep(0.25)
        raise Exception("User did not grant remote access")
    
    def poll_rmmp(self):
        
        # A "persistent session" can only make 400 calls before
        # being disconnected. Once this connection is lost,
        # the grant will be revoked. To work around this,
        # create parallel sessions with copied session cookies
        # to maintain the connection.
        
        url="/".join([self.base_url, 'users/rmmp/poll'])
        
        old_rmmp_session=None
        if self._rmmp_session is None:
            self._do_get(url)
            self._rmmp_session=requests.Session()
            self._rmmp_session_t=time.time()            
            
            for c in self._session.cookies:
                self._rmmp_session.cookies.set_cookie(c) 
        else:
            if time.time() - self._rmmp_session_t > 30:
                old_rmmp_session=self._rmmp_session
                rmmp_session=requests.Session()
                
                for c in self._session.cookies:
                    rmmp_session.cookies.set_cookie(c)
        
        rmmp_session=self._rmmp_session        
                
        res=rmmp_session.get(url, auth=self.auth)
        soup=self._process_response(res)
                
        if old_rmmp_session is not None:
            self._rmmp_session=rmmp_session
            self._rmmp_session_t=time.time()
            try:
                old_rmmp_session.close()
            except:
                pass
        
        return soup.find('span', {'class': 'status'}).text == "GRANTED"

RAPIDExecutionState=namedtuple('RAPIDExecutionState', ['ctrlexecstate', 'cycle'])
RAPIDEventLogEntry=namedtuple('RAPIDEventLogEntry', ['msgtype', 'code', 'tstamp', 'args', 'title', 'desc', 'conseqs', 'causes', 'actions'])
RAPIDIpcMessage=namedtuple('RAPIDIpcMessage',['data','userdef','msgtype','cmd'])
RAPIDSignal=namedtuple('RAPIDSignal',['name','lvalue'])


class ABBException(Exception):
    def __init__(self, message, code):
        super(ABBException, self).__init__(message)
        self.code=code


# rapid = RAPID(base_url='http://192.168.55.1:80')
# while True:
#     rapid.set_digital_io('valve1', 1) 
#     rapid.set_digital_io('valve2', 0) 
#     time.sleep(2)
#     rapid.set_digital_io('valve1', 0) 
#     rapid.set_digital_io('valve2', 1) 
#     time.sleep(2)