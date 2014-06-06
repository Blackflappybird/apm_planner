#include "LinkManager.h"
#include "PxQuadMAV.h"
#include "SlugsMAV.h"
#include "ArduPilotMegaMAV.h"
#include "UASManager.h"
#include "UDPLink.h"
#include <QSettings>
LinkManager::LinkManager(QObject *parent) :
    QObject(parent)
{
    m_mavlinkDecoder = new New_MAVLinkDecoder(this);
    m_mavlinkParser = new New_MAVLinkParser(this);
    m_mavlinkParser->setConnectionManager(this);
    connect(m_mavlinkParser,SIGNAL(messageReceived(LinkInterface*,mavlink_message_t)),m_mavlinkDecoder,SLOT(messageReceived(LinkInterface*,mavlink_message_t)));
    connect(m_mavlinkParser,SIGNAL(protocolStatusMessage(QString,QString)),this,SLOT(protocolStatusMessageRec(QString,QString)));
}
int LinkManager::addSerialConnection()
{
    //Add with defaults
    SerialConnection *connection = new SerialConnection();
    connect(connection,SIGNAL(bytesReceived(LinkInterface*,QByteArray)),m_mavlinkParser,SLOT(receiveBytes(LinkInterface*,QByteArray)));
    m_connectionMap.insert(connection->getId(),connection);
    emit newLink(connection);
    return connection->getId();


}

int LinkManager::addSerialConnection(QString port,int baud)
{
    SerialConnection *connection = new SerialConnection();
    connect(connection,SIGNAL(bytesReceived(LinkInterface*,QByteArray)),m_mavlinkParser,SLOT(receiveBytes(LinkInterface*,QByteArray)));
    connection->setPortName(port);
    connection->setBaudRate(baud);
    m_connectionMap.insert(connection->getId(),connection);
    emit newLink(connection);
    return connection->getId();

}
int LinkManager::addUdpConnection(QHostAddress addr,int port)
{
    UDPLink* udpLink = new UDPLink(addr,port);
    udpLink->connect();
    m_connectionMap.insert(udpLink->getId(),udpLink);
    emit newLink(udpLink);
    return udpLink->getId();

}

void LinkManager::addLink(LinkInterface *link)
{
    m_connectionMap.insert(link->getId(),link);
}
void LinkManager::removeLink(LinkInterface *link)
{
    //if (m_connectionMap.find())
}

void LinkManager::connectLink(int index)
{
    if (m_connectionMap.contains(index))
    {
        m_connectionMap.value(index)->connect();
    }
}

void LinkManager::modifySerialConnection(int index,QString port,int baud)
{

}

void LinkManager::removeSerialConnection(int index)
{

}
void LinkManager::messageReceived(LinkInterface* link,mavlink_message_t message)
{

}
UASInterface* LinkManager::getUas(int id)
{
    if (m_uasMap.contains(id))
    {
        return m_uasMap.value(id);
    }
    return 0;
}
QList<LinkInterface*> LinkManager::getLinks()
{
    QList<LinkInterface*> links;
    for (int i=0;i<m_connectionMap.keys().size();i++)
    {
        links.append(m_connectionMap.value(m_connectionMap.keys().at(i)));
    }
    return links;
}

UASInterface* LinkManager::createUAS(New_MAVLinkParser* mavlink, LinkInterface* link, int sysid, mavlink_heartbeat_t* heartbeat, QObject* parent)
{
    QPointer<QObject> p;

    if (parent != NULL)
    {
        p = parent;
    }
    else
    {
        p = mavlink;
    }

    UASInterface* uas;

    switch (heartbeat->autopilot)
    {
    case MAV_AUTOPILOT_GENERIC:
    {
        UAS* mav = new UAS(0, sysid);
        // Set the system type
        mav->setSystemType((int)heartbeat->type);
        // Connect this robot to the UAS object
        connect(mavlink, SIGNAL(messageReceived(LinkInterface*, mavlink_message_t)), mav, SLOT(receiveMessage(LinkInterface*, mavlink_message_t)));
#ifdef QGC_PROTOBUF_ENABLED
        connect(mavlink, SIGNAL(extendedMessageReceived(LinkInterface*, std::tr1::shared_ptr<google::protobuf::Message>)), mav, SLOT(receiveExtendedMessage(LinkInterface*, std::tr1::shared_ptr<google::protobuf::Message>)));
#endif
        uas = mav;
    }
    break;
    case MAV_AUTOPILOT_PIXHAWK:
    {
        PxQuadMAV* mav = new PxQuadMAV(0, sysid);
        // Set the system type
        mav->setSystemType((int)heartbeat->type);
        // Connect this robot to the UAS object
        // it is IMPORTANT here to use the right object type,
        // else the slot of the parent object is called (and thus the special
        // packets never reach their goal)
        connect(mavlink, SIGNAL(messageReceived(LinkInterface*, mavlink_message_t)), mav, SLOT(receiveMessage(LinkInterface*, mavlink_message_t)));
#ifdef QGC_PROTOBUF_ENABLED
        connect(mavlink, SIGNAL(extendedMessageReceived(LinkInterface*, std::tr1::shared_ptr<google::protobuf::Message>)), mav, SLOT(receiveExtendedMessage(LinkInterface*, std::tr1::shared_ptr<google::protobuf::Message>)));
#endif
        uas = mav;
    }
    break;
    case MAV_AUTOPILOT_SLUGS:
    {
        SlugsMAV* mav = new SlugsMAV(0, sysid);
        // Set the system type
        mav->setSystemType((int)heartbeat->type);
        // Connect this robot to the UAS object
        // it is IMPORTANT here to use the right object type,
        // else the slot of the parent object is called (and thus the special
        // packets never reach their goal)
        connect(mavlink, SIGNAL(messageReceived(LinkInterface*, mavlink_message_t)), mav, SLOT(receiveMessage(LinkInterface*, mavlink_message_t)));
        uas = mav;
    }
    break;
    case MAV_AUTOPILOT_ARDUPILOTMEGA:
    {
        ArduPilotMegaMAV* mav = new ArduPilotMegaMAV(0, sysid);
        // Set the system type
        mav->setSystemType((int)heartbeat->type);
        // Connect this robot to the UAS object
        // it is IMPORTANT here to use the right object type,
        // else the slot of the parent object is called (and thus the special
        // packets never reach their goal)
        connect(mavlink, SIGNAL(messageReceived(LinkInterface*, mavlink_message_t)), mav, SLOT(receiveMessage(LinkInterface*, mavlink_message_t)));
        uas = mav;
    }
    break;
#ifdef QGC_USE_SENSESOAR_MESSAGES
    case MAV_AUTOPILOT_SENSESOAR:
        {
            senseSoarMAV* mav = new senseSoarMAV(0,sysid);
            mav->setSystemType((int)heartbeat->type);
            connect(mavlink, SIGNAL(messageReceived(LinkInterface*, mavlink_message_t)), mav, SLOT(receiveMessage(LinkInterface*, mavlink_message_t)));
            uas = mav;
            break;
        }
#endif
    default:
    {
        UAS* mav = new UAS(0, sysid);
        mav->setSystemType((int)heartbeat->type);
        // Connect this robot to the UAS object
        // it is IMPORTANT here to use the right object type,
        // else the slot of the parent object is called (and thus the special
        // packets never reach their goal)
        connect(mavlink, SIGNAL(messageReceived(LinkInterface*, mavlink_message_t)), mav, SLOT(receiveMessage(LinkInterface*, mavlink_message_t)));
        uas = mav;
    }
    break;
    }

    m_uasMap.insert(sysid,uas);

    // Set the autopilot type
    uas->setAutopilotType((int)heartbeat->autopilot);

    // Make UAS aware that this link can be used to communicate with the actual robot
    uas->addLink(link);

    // Now add UAS to "official" list, which makes the whole application aware of it
    UASManager::instance()->addUAS(uas);

    return uas;

}
void LinkManager::protocolStatusMessageRec(QString title,QString text)
{
    emit protocolStatusMessage(title,text);
    QLOG_DEBUG() << "Protocol Status Message:" << title << text;
}
