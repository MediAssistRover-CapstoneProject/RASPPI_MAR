"""Generate static schemas and examples; opens no network or hardware."""
import copy
import json
import runpy
import sys
from pathlib import Path
from uuid import uuid5,NAMESPACE_URL
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
runpy.run_path(str(ROOT/'tools/export_contract.py'))
from mediassist.step_contract import obj,UUID_SCHEMA,CONTRACT,PARAMS
def put(name,value):
    (ROOT/'handoff'/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def read(name):return json.loads((ROOT/'handoff'/name).read_text(encoding='utf-8'))
session='01234567-89ab-4cde-8123-456789abcdef'
robot='11111111-1111-4111-8111-111111111111'
schema=read('command.schema.json')
schema['properties']['transportSessionId']=UUID_SCHEMA
schema['required'].append('transportSessionId')
put('mqtt-command.schema.json',schema)
envelope=read('pi-envelope.schema.json')
envelope['properties']['transportSessionId']=UUID_SCHEMA
envelope['required'].append('transportSessionId')
put('mqtt-pi-envelope.schema.json',envelope)
responses=[]
for kind,fields in {
 'HELLO_ACK':{'contract':{'const':CONTRACT},'accepted':{'type':'boolean'}},
 'EVENT_ACK':{'messageId':UUID_SCHEMA,'accepted':{'type':'boolean'}},
 'SERVER_HEARTBEAT':{}
}.items():
    responses.append(obj({'messageType':{'const':kind},'robotId':UUID_SCHEMA,'transportSessionId':UUID_SCHEMA,**fields}))
put('mqtt-be-response.schema.json',{'oneOf':responses})
for p in (ROOT/'handoff').glob('example-*.json'):
    value=json.loads(p.read_text())
    value['robotId']=robot;value['transportSessionId']=session
    if 'commandId' in value:
        value['commandId']=str(uuid5(NAMESPACE_URL,'mar-mqtt:'+p.name))
        if 'step' in value:
            value['step']['jobStepId']=str(uuid5(NAMESPACE_URL,'mar-step:'+p.name))
    put(p.name,value)
base=read('example-read-sides.json')
mapping_id='abcdef01-2345-4678-9012-abcdefabcdef'
artifact='12345678-1234-4234-8234-123456789abc'
for action,timeout in (('START',60),('SAVE',90),('STOP',20)):
    value=copy.deepcopy(base)
    value['commandId']=str(uuid5(NAMESPACE_URL,'mapping-'+action))
    s=value['step']
    s.update(jobStepId=str(uuid5(NAMESPACE_URL,'mapping-step-'+action)),stepCode='MAP_'+action,
             operationKey=s['jobId']+':MAP_'+action+':1',timeoutSec=timeout,
             params={'sensing_type':'MAPPING','action':action,'mapping_session_id':mapping_id})
    if action=='SAVE':s['params']['artifact_id']=artifact
    if action=='START':s['params']['preview_rate_hz']=0.2
    put('example-mapping-'+action.lower()+'.json',value)
put('example-server-heartbeat.json',{'messageType':'SERVER_HEARTBEAT','robotId':robot,'transportSessionId':session})
put('topics.json',{'prefix':'mar/robots/{robotId}','commands':{'direction':'BE_TO_PI','qos':1,'retain':False},
    'event-acks':{'direction':'BE_TO_PI','qos':1,'retain':False},'events':{'direction':'PI_TO_BE','qos':1,'retain':False},
    'heartbeat':{'direction':'PI_TO_BE','qos':0,'retain':False},'presence':{'direction':'PI_TO_BE','qos':1,'retain':True},
    'mapping/preview':{'direction':'PI_TO_BE','qos':0,'retain':False}})
print('MQTT schemas exported. UUIDs, session nonce and timestamps in examples are placeholders.')

# The navigation extension does not add a StepType.
nav=read('mqtt-navigation.schema.json')['oneOf']
responses=read('mqtt-be-response.schema.json')
responses['oneOf'].append(nav[0])
put('mqtt-be-response.schema.json',responses)
topics=read('topics.json')
topics['profile-acks']={'direction':'PI_TO_BE','qos':1,'retain':False}
for lane,direction in [('mapping/control','BE_TO_PI'),('mapping/control-acks','PI_TO_BE'),('camera/preview','PI_TO_BE')]:
    topics[lane]={'direction':direction,'qos':0,'retain':False}
put('topics.json',topics)
