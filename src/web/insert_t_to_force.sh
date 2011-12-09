#!/bin/bash

#run soap login to get session id

API_URL="https://na12.salesforce.com/services/data/v23.0/sobjects/temperature__c"
SESSION_ID=
JSON_RECORD=

echo $#

usage()
{
     echo "insert_t_to_force"
     echo "usage: insert_t_to_force.sh sensorId temperature)"
     echo "inserts temperature sensor reading into salesforce app server"
}

login()
{
    SESSION_ID=`curl https://login.salesforce.com/services/Soap/u/23.0 \
	-H "Content-Type:text/xml; charset=UTF-8" -H "SOAPAction: login" -d @login.txt | sed -e \
	"s/.*<sessionId>\(.*\)<\/sessionId>.*/\1/"`
}

compose_json()
{
    JSON_RECORD="{ \"Name\": \"temperature\",\
                   \"SensorId__c\":\"$1\", \
                   \"temp__c\":\"$2\"}"
}

send()
{
    echo inserting "$JSON_RECORD"
    curl "$API_URL" -H "Content-Type:application/json" -d "$JSON_RECORD" -H "Authorization: OAuth $SESSION_ID" -H "X-PrettyPrint:1"
}

if [ $# -ne 2 ]; then
  usage
  exit
fi

compose_json $1 $2

if [ -z $SESSION_ID ]; then
    echo getting session id
    login
fi

send $JSON_RECORD


