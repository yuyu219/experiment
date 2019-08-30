//���]�s���@�䥢��, �N������, �|�^��ack
#include<iostream>
#include<fstream>
#include<ctime>
#include<cstdlib>
#include<cstring>
#include<string>
#include<vector>
#include <algorithm>
#include <typeinfo>
using namespace std;

const int queueSize = 100, intervalTime = 20000000, runningTime = 100;	//�̧Ǭ�: �w�ľ��j�p, �����`�ɶ�, ���禸��
const int SIFS = 3, SlotTime = 5, DIFS = 13, EIFS = 16;
const double transmission_delay_data = 90.15385, propagation_delay = 0.1668, Wait_ACK_Time = 94.026, transmission_delay_ACK = 0.5385;
const int CW[7] = { 16,32,64,128,256,512,1024 };
const int nodeNum = 8;	//�`�I�ƶq
const int SendTimesNum = (nodeNum - 1) * 2;
  //�`�ѽX���~����, �`�s�X�s�����\����, �`�ѽX����,    �s�X��I������,   �`���T�ضq, DA���ѽX���~����, DA���s�X�s�����\����, DA���s�X�I������, �Ψӭp��U�`�I�����T�ضq, �Q���ǿ骺�`����,   B�I�Q���ǿ�o����
int decodeError = 0, EncodeTimes = 0, DecodeTimes = 0, EncodeCollision = 0, Throw = 0, DADecodeError = 0, DABroadcast = 0, DABroadcastCol = 0, nodeThrowNum[nodeNum] = { 0 }, inhibition_total = 0, inhibition_B = 0;
int nodeseq[nodeNum] = { 0, 1, 2, 3, 4, 5, 6, 7 };	//���H�`�I�ƶq����
//�ʥ]
struct Packet {
	string type;    //RTS, CTS, ACK, Data
	string cla;   //��ګʥ]�S��, ������Frame comtrol�����, �i�H��{�ʥ]���O ex.nonNC, twobuffer, RC-twobuffer(our)
	int seq;    //��ګʥ]�S��, ��K�T�{��,�ʥ]�Ǹ�,�����O�p�� , ex.p1, q2, p3. ��ڤW�x�s��1, 2, 3

	int Duration;   //RTS, CTS,
	int TA, SA, DA, RA, ThrowNum; //0=A, 1=B, 2=C
	int qseq, qSA, qDA, qRA, qThrowNum;
};
//RTS: Duration, TA, RA;
//CTS: Duration, RA;
//ACK: Duration, RA;
//�p���
struct CountTimes {
	int CountType[3];  //CountType: 0 send, 1 receive, 2 collision
};
//node
class Node {
public:
	int number, retryTimes, StartTimes;
	double backoffTime;
	vector <Packet> outputQueue, decodeQueue;
	vector <int> neighborNode;

	Node() { number = 0; };
	Node(int a);

	//void Retry(Node SendNode);   //�ʥ]�Ĭ𭫶Ǯɰ���, retryTimes�MbackoffTime�n���]
	void Retry();   //�ʥ]�Ĭ𭫶Ǯɰ���, retryTimes�MbackoffTime�n���]
	void Reset();   //�o�e�s�ʥ]��

	void AddPacket(Packet p);   //�i��O������ʥ]�ɩI�s�Ϊ̬O�C���s�W�ʥ]�ɩI�s
};
class TimePoint {
public:
	int Send_N;
	double Start_T, backoff_T;
	string DoWhat;  //Send_Coll or Receive
};
class sort_indices{
private:
	const vector <int> &mparr;
public:
	sort_indices(const vector <int> &parr) : mparr(parr) {}
	bool operator()(int i, int j) const { return mparr[i] < mparr[j]; }
};

Node SendPacket(vector<Node> &lists, vector<TimePoint> &listTime, int index);   //�ǰe�ʥ]
int collission(vector<Node> &lists, vector<TimePoint> &listTime, Node nodes[nodeNum], Node SendNode, int z, CountTimes SendTimes[][2]);
void ReceivePacket(Node nodes[nodeNum], Node &SendNode, vector<Node> &lists, vector<TimePoint> &listTime);   //�^�Ǳ����`�I
void Count(CountTimes SendTimes[][2], Node SendNode, int CountType);  //CountType: 0 send, 1 receive, 2 collision

void Encode(Node &SendNode);
void Decode(Node nodes[nodeNum], Node &SendNode, Packet &after, int receiveNode);
Packet DecodeResultP(Packet r, Packet p, char pa);
void PushToQueue(vector <Packet> &Que, Packet pushPacket);
void IntermediateNode(Packet &ReceiveP, Node nodes[nodeNum], vector<Node> &lists, vector<TimePoint> &listTime);

Packet add_p(int seq);
Packet add_q(int seq);
Packet add_ack(int RA);
Packet errorPkt();
bool cmp(Node a, Node b);
bool cmp2(TimePoint a, TimePoint b);

int main()
{
	ofstream fout("DBS-8node-siz100-100.txt");
	if (!fout) {
		fout << "�L�k�g�J�ɮ�\n";
		return 1;
	}
	int n = 1;
	int i, j;
	srand((unsigned)time(NULL));

	while (n <= runningTime) {  //���禸��
		(double)rand() / RAND_MAX;
		Node nodes[nodeNum] = { 0,1,2,3,4,5,6,7 };  //0123=ABCD, ���ǹ��
		vector <Node> lists;    //�Ƨǥ�
		vector <TimePoint> listTime;    //�Ƨǥ�
		int TimeRange = 1000000;
		EncodeTimes = 0;
		DecodeTimes = 0;
		EncodeCollision = 0;
		Throw = 0;
		DABroadcast = 0;
		DABroadcastCol = 0;
		inhibition_total = 0;
		inhibition_B = 0;
		nodes[0].neighborNode.push_back(1);

		nodes[1].neighborNode.push_back(0);
		nodes[1].neighborNode.push_back(2);

		nodes[2].neighborNode.push_back(1);
		nodes[2].neighborNode.push_back(3);

		nodes[3].neighborNode.push_back(2);
		nodes[3].neighborNode.push_back(4);

		nodes[4].neighborNode.push_back(3);
		nodes[4].neighborNode.push_back(5);

		nodes[5].neighborNode.push_back(4);
		nodes[5].neighborNode.push_back(6);

		nodes[6].neighborNode.push_back(5);
		nodes[6].neighborNode.push_back(7);

		nodes[7].neighborNode.push_back(6);

		CountTimes SendTimes[SendTimesNum][2];    //AtoB, BtoC, CtoD, DtoE, EtoD, DtoC, CtoB, BtoA
					//Data0, ACK1

		int packetSeq = 0;    //�x�s�ʥ]�s��
		//���G��l��
		for (i = 0; i < SendTimesNum; i++)
			for (j = 0; j < 2; j++)       //0: Data, 1: ACK
				for (int k = 0; k < 3; k++)     //CountType: 0 send, 1 receive, 2 collision
					SendTimes[i][j].CountType[k] = 0;
		decodeError = 0;
		DADecodeError = 0;
		for (i = 0; i < nodeNum; i++)
			nodeThrowNum[i] = 0;

		packetSeq++;
		Packet p = add_p(packetSeq);
		nodes[0].AddPacket(p);
		Packet q = add_q(packetSeq);
		nodes[nodeNum - 1].AddPacket(q);
		for (double z = 0; z < intervalTime;) { //����ɶ�

			for (i = 0; i < nodeNum; i++) { //��J���
				if (nodes[i].outputQueue.size() > 0) {    //�p�G�ݭn�e�ʥ]�B�b�ɶ��d��
					bool isExist = false;
					for (j = 0; j < listTime.size(); j++) {
						if (nodes[i].number == listTime[j].Send_N) {
							if (listTime[j].DoWhat == "Send_Collision") {
								lists[j].outputQueue = nodes[i].outputQueue;
								lists[j].decodeQueue = nodes[i].decodeQueue;
							}
							isExist = true;
						}
					}
					if (!isExist) {
						TimePoint Point;
						Point.Send_N = i;
						Point.backoff_T = nodes[i].backoffTime;
						Point.DoWhat = "Send_Collision";
						if (nodes[i].outputQueue[0].type == "ACK")
							Point.Start_T = nodes[i].StartTimes + SIFS + nodes[i].backoffTime;
						else
							Point.Start_T = nodes[i].StartTimes + DIFS + nodes[i].backoffTime;
						listTime.push_back(Point);

						Node SN = nodes[i];
						SN.StartTimes = Point.Start_T;
						lists.push_back(SN);
					}
				}
			}
			/*cout << z << endl;
			cout << "node\tout\tde\tbackoff\tstart\tretry" << endl;
			for (i = 0; i < nodeNum; i++) {
				cout << i << "\t" << nodes[i].outputQueue.size() << "\t" << nodes[i].decodeQueue.size() << "\t" << nodes[i].backoffTime << "\t" << nodes[i].StartTimes << "\t" << nodes[i].retryTimes << endl;
			}

			for (i = 0; i < nodeNum; i++) {
				cout << i << "\t";
				for (j = 0; j < nodes[i].outputQueue.size() && j < 5; j++) {
					cout << nodes[i].outputQueue[j].cla << " " << nodes[i].outputQueue[j].seq << " " << nodes[i].outputQueue[j].ThrowNum << "\t";
				}
				cout << endl;
				cout << i << "\t";
				for (j = 0; j < nodes[i].decodeQueue.size() && j < 5; j++) {
					cout << nodes[i].decodeQueue[j].cla << " " << nodes[i].decodeQueue[j].seq << " " << nodes[i].decodeQueue[j].ThrowNum << "\t";
				}
				cout << endl;
			}
			for (i = 0; i < listTime.size(); i++) {
				cout << listTime[i].Start_T << "\t" << lists[i].StartTimes << "\t" << lists[i].number << "\t"
					<< listTime[i].DoWhat << "\t" << lists[i].outputQueue[0].type << endl;
			}*/
			if (listTime.size() == 0) {    //�j�a���b���ݦ^�_, �Y�L�H�i�e
				z = z + 1;
				continue;
			}
			int *indices = new int[lists.size()];
			vector <int> StartT;
			for (i = 0; i < lists.size(); i++)
			{
				indices[i] = i;
				StartT.push_back(listTime[i].Start_T);
			}
			std::sort(indices, indices + lists.size(), sort_indices(StartT));
			/*cout << "--------------Sort-------------------" << endl;
			for (i = 0; i < lists.size(); i++)
			{
				cout << StartT[indices[i]] << "\t" << listTime[indices[i]].Start_T << "\t" << lists[indices[i]].StartTimes << "\t" << lists[indices[i]].number << "\t"
					<< listTime[indices[i]].DoWhat << "\t" << lists[indices[i]].outputQueue[0].type << endl;
			}
			cout << endl;*/
			z = listTime[indices[0]].Start_T;
			Node SendNode(0);
			if (listTime[indices[0]].DoWhat == "Send_Collision") {
				//�ǰe�B�z
				SendNode = SendPacket(lists, listTime, indices[0]);

				//�p��ǰe����
				Count(SendTimes, SendNode, 0);
				//�I������
				if (collission(lists, listTime, nodes, SendNode, z, SendTimes)) {   //���I��
					if (SendNode.outputQueue[0].cla == "NC") {
						EncodeCollision++;
						if (SendNode.number == 1 or SendNode.number == nodeNum - 2)
							DABroadcastCol++;
					}
					//cout << "collission" << endl;
					nodes[SendNode.number].Retry();
					//�p��ǰe���Ѫ�����
					Count(SendTimes, SendNode, 2);
				}
				else {   //���\�ǿ�
					//cout << "SendNode: " << SendNode.number << endl;
					//cout << SendNode.outputQueue[0].cla << "\t" << SendNode.outputQueue[0].seq << "\t" << SendNode.outputQueue[0].ThrowNum << endl;
					if (SendNode.outputQueue[0].cla == "NC") {
						EncodeTimes++;
						if (SendNode.number == 1 or SendNode.number == nodeNum - 2)
							DABroadcast++;
					}
					//�[�W�ǰe�ɶ�
					if (SendNode.outputQueue[0].type == "Data") {    //�p�G�ODATA�n���ݦ^�_, ack�h����
						nodes[SendNode.number].StartTimes = SendNode.StartTimes + Wait_ACK_Time;
						nodes[SendNode.number].Reset();
					}
					else {   //�p�G�OACK
						//�ݦ��`�I�O�_�����A(data�ʥ])�w�s�ilist, �����ܥΥL����ƨS�����ܴNreset
						int isPush = false;
						for (j = 0; j < lists.size(); j++) {
							if (lists[j].number == SendNode.number) {	//�p�G�w�g�s�blists�̭�, �ϥ�lists���Ѽ�

								nodes[SendNode.number].StartTimes = lists[j].StartTimes - DIFS - lists[j].backoffTime;	//�קK����startTime�p����~
								isPush = true;
							}
						}
						if (!isPush) {	//�p�G���s�blists�̭�, �nreset, �BstartTime�n��(�קK�@�}�l�S�����ɭ�start�ȫܤp)
							nodes[SendNode.number].Reset();
							nodes[SendNode.number].StartTimes = SendNode.StartTimes;
						}
					}
					/*cout << "list size: " << lists.size() << endl;
					cout << "push Receive to lists" << endl;*/

					TimePoint Point;
					Point.Send_N = SendNode.number;
					Point.backoff_T = SendNode.backoffTime;
					Point.DoWhat = "Receive";

					if (SendNode.outputQueue[0].type == "ACK")
						Point.Start_T = z + transmission_delay_ACK + propagation_delay;
					else
						Point.Start_T = z + transmission_delay_data + propagation_delay;
					listTime.push_back(Point);

					Node SN = SendNode;
					SN.StartTimes = Point.Start_T;
					lists.push_back(SN);

					nodes[SendNode.number].outputQueue = SendNode.outputQueue;
					nodes[SendNode.number].decodeQueue = SendNode.decodeQueue;
					nodes[SendNode.number].outputQueue.erase(nodes[SendNode.number].outputQueue.begin());
				}
			}
			else {   //Receive
				//cout << "Receive" << endl;
				SendNode = lists[indices[0]];

				lists.erase(lists.begin() + indices[0]);
				listTime.erase(listTime.begin() + indices[0]);
				//�p�Ⱶ������
				Count(SendTimes, SendNode, 1);
				//����
				ReceivePacket(nodes, SendNode, lists, listTime);
			}
			if (nodes[0].outputQueue.size() < 100 || nodes[nodeNum - 1].outputQueue.size() < 100) {
				packetSeq++;
				Packet p = add_p(packetSeq);
				nodes[0].AddPacket(p);
				Packet q = add_q(packetSeq);
				nodes[nodeNum - 1].AddPacket(q);
			}
			vector <Packet>().swap(SendNode.outputQueue);
			vector <Packet>().swap(SendNode.decodeQueue);

			/*for (i = 1; i < nodeNum-1; i++) {
				cout << i << "\t";
				for (j = 0; j < nodes[i].outputQueue.size(); j++) {
					cout << nodes[i].outputQueue[j].cla << "\t";
				}
				cout << endl;
			}*/

			if (z >= TimeRange) {
				cout << z << endl;
				for (i = 0; i < 3; i++) {   //i: 0 send, 1 receive, 2 collision
					for (j = 0; j < 2; j++) {   //j: 0 Data, 1 ACK
						for (int k = 0; k < SendTimesNum; k++) {
							fout << SendTimes[k][j].CountType[i] << "\t";
						}
					}
				}
				fout << EncodeTimes << "\t" << DecodeTimes << "\t" << decodeError << "\t" << EncodeCollision << "\t" << Throw << "\t" << DADecodeError << "\t" << DABroadcast << "\t" << DABroadcastCol << endl;

				for (i = 0; i < nodeNum; i++) {
					cout << nodes[i].outputQueue.size() << "\t";
				}
				cout << endl;
				for (i = 0; i < nodeNum; i++) {
					cout << nodeThrowNum[i] << "\t";
				}
				cout << endl;

				TimeRange += 1000000;
			}
			//cout << endl;
			delete[] indices;
		}
		cout << "round " << n << endl;
		for (int i = 0; i < nodeNum; i++) {
			vector <Packet>().swap(nodes[i].outputQueue);
			vector <Packet>().swap(nodes[i].decodeQueue);
		}
		vector <Node>().swap(lists);
		vector <TimePoint>().swap(listTime);

		n++;
	}
	fout.close();
	system("pause");
	return 0;
}
//class
Node::Node(int a) {
	number = a;
	StartTimes = 0;
	retryTimes = 0;
	double randomNum = (double)rand() / RAND_MAX;
	backoffTime = (randomNum * CW[retryTimes]) * SlotTime;
}
void Node::Retry() {
	StartTimes = StartTimes + DIFS + backoffTime + Wait_ACK_Time; //�n�����O�ۤv��backoff time
	if (retryTimes < 6)
		retryTimes++;
	double randomNum = (double)rand() / RAND_MAX;
	backoffTime = (randomNum * CW[retryTimes]) * SlotTime;
}
void Node::Reset() {
	retryTimes = 0;
	double randomNum = (double)rand() / RAND_MAX;
	backoffTime = (randomNum * CW[retryTimes]) * SlotTime;
}
void Node::AddPacket(Packet p) {
	outputQueue.push_back(p);
}
Node SendPacket(vector<Node> &lists, vector<TimePoint> &listTime, int index) {
	Node SendNode = lists[index];
	//�s�X
	Encode(SendNode);
	SendNode.outputQueue[0].TA = SendNode.number;
	lists.erase(lists.begin() + index);
	listTime.erase(listTime.begin() + index);
	return SendNode;
}
//int collission(vector<Node>& lists, vector<TimePoint>& listTime, Node nodes[nodeNum], Node SendNode, int z, CountTimes AtoB[], CountTimes BtoC[], CountTimes CtoD[], CountTimes DtoC[], CountTimes CtoB[], CountTimes BtoA[]) {
int collission(vector<Node>& lists, vector<TimePoint>& listTime, Node nodes[nodeNum], Node SendNode, int z, CountTimes SendTimes[][2]) {
	bool isCollossion = false;
	for (int i = 0; i < listTime.size(); i++) {
		if (SendNode.outputQueue[0].type == "Data" && lists[i].outputQueue[0].type == "Data" && listTime[i].DoWhat == "Send_Collision") {
			if (lists[i].StartTimes < SendNode.StartTimes + (Wait_ACK_Time - propagation_delay)) {    //�i��I��
				//�I���F
				if (count(SendNode.neighborNode.begin(), SendNode.neighborNode.end(), lists[i].number) != 0) {	//list[i]�OSendNode���F�~
					//�e�ΦP�@���W�D���ܭn�B�zDCF
					if (lists[i].StartTimes - SendNode.StartTimes < lists[i].backoffTime)
						lists[i].backoffTime = lists[i].StartTimes - SendNode.StartTimes;

					lists[i].StartTimes = SendNode.StartTimes + Wait_ACK_Time + DIFS + lists[i].backoffTime;
					listTime[i].backoff_T = lists[i].backoffTime;
					listTime[i].Start_T = lists[i].StartTimes;

					nodes[lists[i].number].backoffTime = lists[i].backoffTime;
					nodes[lists[i].number].StartTimes = SendNode.StartTimes + Wait_ACK_Time;
					inhibition_total++;
					if (lists[i].outputQueue[0].cla == "NC")
						inhibition_B++;
				}
				//�o�̤��|�i�Ӧ]��DCF(�Ĥ@��if�ױ��F)���****
				else if ((SendNode.number == lists[i].outputQueue[0].RA || SendNode.number == lists[i].outputQueue[0].qRA) && SendNode.outputQueue[0].RA != lists[i].number && SendNode.outputQueue[0].qRA != lists[i].number) {
					//                                  list[i]�n�e���ǰe�I                                          �B                                ���ݩ�ǰe�I�n�e���P�@���W�D
					//cout << "collision1 " << lists[i].number << endl;
					isCollossion = true;
					Count(SendTimes, nodes[lists[i].number], 0);
					Count(SendTimes, nodes[lists[i].number], 2);
					if (lists[i].outputQueue[0].cla == "NC") {
						EncodeCollision++;
						if (lists[i].number == 1 or lists[i].number == nodeNum - 2)
							DABroadcastCol++;
					}
					nodes[lists[i].number].Retry();
					listTime.erase(listTime.begin() + i);
					lists.erase(lists.begin() + i);
					i--;
				}
				//�o�̤��|�i�Ӧ]��DCF(�Ĥ@��if�ױ��F)���****
				else if ((lists[i].number == SendNode.outputQueue[0].RA || lists[i].number == SendNode.outputQueue[0].qRA) && lists[i].outputQueue[0].RA != SendNode.number && lists[i].outputQueue[0].qRA != SendNode.number) {
					//                                             �ǰe�I�n�e��list[i]                                     �B                                ���ݩ�ǰe�I�n�e���P�@���W�D
					//cout << "collision2 " << lists[i].number << endl;
					isCollossion = true;
					Count(SendTimes, nodes[lists[i].number], 0);
					Count(SendTimes, nodes[lists[i].number], 2);
					if (lists[i].outputQueue[0].cla == "NC") {
						EncodeCollision++;
						if (lists[i].number == 1 or lists[i].number == nodeNum - 2)
							DABroadcastCol++;
					}
					nodes[lists[i].number].Retry();
					listTime.erase(listTime.begin() + i);
					lists.erase(lists.begin() + i);
					i--;
				}
				else if (lists[i].outputQueue[0].RA == SendNode.outputQueue[0].RA || lists[i].outputQueue[0].RA == SendNode.outputQueue[0].qRA || lists[i].outputQueue[0].qRA == SendNode.outputQueue[0].RA
					|| (lists[i].outputQueue[0].cla == "NC" && SendNode.outputQueue[0].cla == "NC" && lists[i].outputQueue[0].qRA == SendNode.outputQueue[0].qRA)) {    //���ONC�~�i�H��qRA
			// 								                    �ǰe�I��RA��lists[i]�@��
					//cout << "collision3 " << lists[i].number << endl;
					isCollossion = true;
					Count(SendTimes, nodes[lists[i].number], 0);
					Count(SendTimes, nodes[lists[i].number], 2);
					if (lists[i].outputQueue[0].cla == "NC") {		//�o�̤��|�i��, �]���O�ǰe�e�~�s�X��, �ҥHlist�����I�����|��NC���T��*****
						EncodeCollision++;
						if (lists[i].number == 1 or lists[i].number == nodeNum - 2)
							DABroadcastCol++;
					}
					nodes[lists[i].number].Retry();
					listTime.erase(listTime.begin() + i);
					lists.erase(lists.begin() + i);
					i--;
				}
				else {	//�S���I���n�[�^�h
					if (SendNode.outputQueue[0].type == "ACK") {	//Send��Data
						lists[i].backoffTime = lists[i].backoffTime + transmission_delay_data + SendNode.backoffTime + propagation_delay;
					}
					else {	//Send��ACK
						lists[i].backoffTime = lists[i].backoffTime + transmission_delay_data + SendNode.backoffTime + transmission_delay_ACK;
					}
					listTime[i].backoff_T = lists[i].backoffTime;
				}
			}
		}
	}
	return isCollossion;
}
void ReceivePacket(Node nodes[nodeNum], Node & SendNode, vector<Node>& lists, vector<TimePoint>& listTime) { //�w�T�w�ORA
	Packet p = SendNode.outputQueue[0];
	int ReceiveNodeNum = p.RA;

	if (p.type == "ACK" && p.RA == nodes[ReceiveNodeNum].number) {   //ack, �s���S��
	}
	else if (p.DA == p.RA || p.qDA == p.qRA) {  //RA�ODA, �i��O�s�X�Υ��s�X
		if (p.cla == "NC") {
			Packet afterDecodeP1, afterDecodeP2;
			Decode(nodes, SendNode, afterDecodeP1, p.RA);
			Decode(nodes, SendNode, afterDecodeP2, p.qRA);

			if (afterDecodeP1.cla == "err") {
				decodeError++;
			}
			else {
				if (p.DA != p.RA) {
					IntermediateNode(afterDecodeP1, nodes, lists, listTime);
					if (nodes[p.RA].outputQueue.size() >= queueSize) {
						nodeThrowNum[p.RA]++;
						Throw++;
					}
					PushToQueue(nodes[p.RA].outputQueue, afterDecodeP1);
				}
				else {
					Packet ack = add_ack(p.TA);
					//cout << SendNode.outputQueue[0].RA << endl;
					//cout << nodes[SendNode.outputQueue[0].RA].outputQueue.size() << endl;
					nodes[SendNode.outputQueue[0].RA].outputQueue.insert(nodes[SendNode.outputQueue[0].RA].outputQueue.begin(), ack);
					//cout << nodes[SendNode.outputQueue[0].RA].outputQueue.size() << endl;

					Node A = nodes[SendNode.outputQueue[0].RA];
					A.backoffTime = 0;
					A.StartTimes = nodes[p.TA].StartTimes - Wait_ACK_Time + transmission_delay_data + SIFS + 1;
					lists.push_back(A);

					TimePoint TP;
					TP.backoff_T = A.backoffTime;
					TP.Send_N = SendNode.outputQueue[0].RA;
					TP.Start_T = A.StartTimes;
					TP.DoWhat = "Send_Collision";
					listTime.push_back(TP);
					/*cout << "ACK1 " << SendNode.outputQueue[0].qRA << endl;
					cout << ack.type << "\t" << ack.cla << "\t" << ack.Duration << endl;
					cout << ack.seq << "\t" << ack.SA << "\t" << ack.DA << "\t" << ack.RA << "\t" << ack.ThrowNum << "\t" << ack.TA << endl;
					cout << ack.qseq << "\t" << ack.qSA << "\t" << ack.qDA << "\t" << ack.qRA << "\t" << ack.qThrowNum << endl;*/
				}
			}

			if (afterDecodeP2.cla == "err") {
				decodeError++;
			}
			else {
				if (p.qDA != p.qRA) {
					IntermediateNode(afterDecodeP2, nodes, lists, listTime);
					if (nodes[p.qRA].outputQueue.size() >= queueSize) {
						nodeThrowNum[p.qRA]++;
						Throw++;
					}
					PushToQueue(nodes[p.qRA].outputQueue, afterDecodeP2);
				}
				else {
					Packet ack = add_ack(p.TA);
					//cout << SendNode.outputQueue[0].qRA << endl;
					//cout << nodes[SendNode.outputQueue[0].qRA].outputQueue.size() << endl;
					nodes[SendNode.outputQueue[0].qRA].outputQueue.insert(nodes[SendNode.outputQueue[0].qRA].outputQueue.begin(), ack);
					//cout << nodes[SendNode.outputQueue[0].qRA].outputQueue.size() << endl;

					Node A = nodes[SendNode.outputQueue[0].qRA];
					A.backoffTime = 0;
					A.StartTimes = nodes[p.TA].StartTimes - Wait_ACK_Time + transmission_delay_data + SIFS + 1;
					lists.push_back(A);

					TimePoint TP;
					TP.backoff_T = A.backoffTime;
					TP.Send_N = SendNode.outputQueue[0].qRA;
					TP.Start_T = A.StartTimes;
					TP.DoWhat = "Send_Collision";
					listTime.push_back(TP);
					/*cout << "ACK2 " << SendNode.outputQueue[0].qRA << endl;
					cout << ack.type << "\t" << ack.cla << "\t" << ack.Duration << endl;
					cout << ack.seq << "\t" << ack.SA << "\t" << ack.DA << "\t" << ack.RA << "\t" << ack.ThrowNum << "\t" << ack.TA << endl;
					cout << ack.qseq << "\t" << ack.qSA << "\t" << ack.qDA << "\t" << ack.qRA << "\t" << ack.qThrowNum << endl;*/

				}

			}
			/*cout << p.type << "\t" << p.cla << "\t" << p.Duration << endl;
			cout << p.seq << "\t" << p.SA << "\t" << p.DA << "\t" << p.RA << "\t" << p.ThrowNum << "\t" << p.TA << endl;
			cout << p.qseq << "\t" << p.qSA << "\t" << p.qDA << "\t" << p.qRA << "\t" << p.qThrowNum << endl;

			cout << afterDecodeP1.type << "\t" << afterDecodeP1.cla << "\t" << afterDecodeP1.Duration << endl;
			cout << afterDecodeP1.seq << "\t" << afterDecodeP1.SA << "\t" << afterDecodeP1.DA << "\t" << afterDecodeP1.RA << "\t" << afterDecodeP1.ThrowNum << "\t" << afterDecodeP1.TA << endl;
			cout << afterDecodeP1.qseq << "\t" << afterDecodeP1.qSA << "\t" << afterDecodeP1.qDA << "\t" << afterDecodeP1.qRA << "\t" << afterDecodeP1.qThrowNum << endl;

			cout << afterDecodeP2.type << "\t" << afterDecodeP2.cla << "\t" << afterDecodeP2.Duration << endl;
			cout << afterDecodeP2.seq << "\t" << afterDecodeP2.SA << "\t" << afterDecodeP2.DA << "\t" << afterDecodeP2.RA << "\t" << afterDecodeP2.ThrowNum << "\t" << afterDecodeP2.TA << endl;
			cout << afterDecodeP2.qseq << "\t" << afterDecodeP2.qSA << "\t" << afterDecodeP2.qDA << "\t" << afterDecodeP2.qRA << "\t" << afterDecodeP2.qThrowNum << endl;
			for (int i = 0; i < listTime.size(); i++) {
				cout << listTime[i].Start_T << "\t" << lists[i].StartTimes << "\t" << lists[i].number << "\t"
					<< listTime[i].DoWhat << "\t" << lists[i].outputQueue[0].type << endl;
			}*/
		}
		else {
			Packet ack = add_ack(p.TA);
			nodes[ReceiveNodeNum].outputQueue.insert(nodes[ReceiveNodeNum].outputQueue.begin(), ack);

			Node A = nodes[ReceiveNodeNum];
			A.backoffTime = 0;
			A.StartTimes = nodes[p.TA].StartTimes - Wait_ACK_Time + transmission_delay_data + SIFS;
			lists.push_back(A);

			TimePoint TP;
			TP.backoff_T = A.backoffTime;
			TP.Send_N = ReceiveNodeNum;
			TP.Start_T = A.StartTimes;
			TP.DoWhat = "Send_Collision";
			listTime.push_back(TP);
			/*cout << p.type << "\t" << p.cla << "\t" << p.Duration << endl;
			cout << p.seq << "\t" << p.SA << "\t" << p.DA << "\t" << p.RA << "\t" << p.ThrowNum << "\t" << p.TA << endl;
			cout << p.qseq << "\t" << p.qSA << "\t" << p.qDA << "\t" << p.qRA << "\t" << p.qThrowNum << endl;*/
			//�R���W�@�Ӷǰe�`�I��decode queue
			for (int i = 0; i < nodes[SendNode.number].neighborNode.size(); i++) {
				bool isDel = false;
				for (int j = 0; j < nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue.size(); j++) {
					if (nodes[nodes[SendNode.number].neighborNode[i]].number != nodes[ReceiveNodeNum].number &&	//���ର�����`�I
						nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue[j].SA == p.SA &&
						nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue[j].DA == p.DA) {

						while (nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue[j].ThrowNum > 0 && p.ThrowNum > 0) {
							p.ThrowNum--;
							nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue[j].ThrowNum--;
						}
						if (p.ThrowNum > 0) {
							nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue.erase(nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue.begin() + j);
							j--;
							p.ThrowNum--;
						}
						else {
							if (nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue[j].ThrowNum == 0) {   //�n�ѽX���Q�R���F
								nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue.erase(nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue.begin() + j);
							}
							isDel = true;
							break;
						}
					}
				}
				if (isDel)
					break;
			}
		}
	}
	else {   //�����`�I
		if (p.cla == "NC") {
			Packet afterDecodeP1, afterDecodeP2;
			Decode(nodes, SendNode, afterDecodeP1, p.RA);
			Decode(nodes, SendNode, afterDecodeP2, p.qRA);

			//nodes[SendNode.number].outputQueue.erase(nodes[SendNode.number].outputQueue.begin());
			if (afterDecodeP1.cla == "err") {
				decodeError++;
			}
			else {
				IntermediateNode(afterDecodeP1, nodes, lists, listTime);
				if (nodes[p.RA].outputQueue.size() >= queueSize) {
					nodeThrowNum[p.RA]++;
					Throw++;
				}
				PushToQueue(nodes[p.RA].outputQueue, afterDecodeP1);
			}

			if (afterDecodeP2.cla == "err") {
				decodeError++;
			}
			else {
				IntermediateNode(afterDecodeP2, nodes, lists, listTime);
				if (nodes[p.qRA].outputQueue.size() >= queueSize) {
					nodeThrowNum[p.qRA]++;
					Throw++;
				}
				PushToQueue(nodes[p.qRA].outputQueue, afterDecodeP2);
			}

		}
		else {
			IntermediateNode(p, nodes, lists, listTime);
			//�R���W�@�Ӷǰe�`�I��decode queue
			for (int i = 0; i < nodes[SendNode.number].neighborNode.size(); i++) {
				bool isDel = false;
				for (int j = 0; j < nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue.size(); j++) {
					if (nodes[nodes[SendNode.number].neighborNode[i]].number != nodes[ReceiveNodeNum].number &&	//���ର�����`�I
						nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue[j].SA == p.SA &&
						nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue[j].DA == p.DA) {
						while (nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue[j].ThrowNum > 0 && p.ThrowNum > 0) {
							p.ThrowNum--;
							nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue[j].ThrowNum--;
						}
						if (p.ThrowNum > 0) {
							nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue.erase(nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue.begin() + j);
							j--;
							p.ThrowNum--;
						}
						else {
							if (nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue[j].ThrowNum == 0) {   //�n�ѽX���Q�R���F
								nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue.erase(nodes[nodes[SendNode.number].neighborNode[i]].decodeQueue.begin() + j);
							}
							isDel = true;
							break;
						}
					}
				}
				if (isDel)
					break;
			}
			p.ThrowNum = 0;
			if (nodes[ReceiveNodeNum].outputQueue.size() >= queueSize) {
				nodeThrowNum[ReceiveNodeNum]++;
				Throw++;
			}
			PushToQueue(nodes[ReceiveNodeNum].outputQueue, p);
			PushToQueue(nodes[SendNode.number].decodeQueue, p);
		}
	}
}
void Count(CountTimes SendTimes[][2], Node SendNode, int CountType) {  //CountType: 0 send, 1 receive, 2 collision
	Packet p = SendNode.outputQueue[0];
	int Type = 0;
	if (p.type == "ACK") {
		Type = 1;
	}
	for (int x = 1; x < nodeNum - 1; x++) {   //RA
		if (p.RA == x) {
			if (SendNode.number == x - 1)
				SendTimes[x - 1][Type].CountType[CountType]++;    //�p��j, ex AtoB, BtoC, CtoD
			else if (SendNode.number == x + 1)
				SendTimes[SendTimesNum - x - 1][Type].CountType[CountType]++;    //�j��p, ex DtoC, CtoB, BtoA
			else
				cout << "Count false" << endl;
		}
		if (p.qRA == x) {
			if (SendNode.number == x - 1)
				SendTimes[x - 1][Type].CountType[CountType]++;    //�p��j, ex AtoB, BtoC, CtoD
			else if (SendNode.number == x + 1)
				SendTimes[SendTimesNum - x - 1][Type].CountType[CountType]++;    //�j��p, ex DtoC, CtoB, BtoA
			else
				cout << "Count false" << endl;
		}
	}
	if (p.RA == 0)
		SendTimes[SendTimesNum - 1][Type].CountType[CountType]++;    //�p��j, ex AtoB, BtoC, CtoD
	else if (p.RA == nodeNum - 1)
		SendTimes[nodeNum - 2][Type].CountType[CountType]++;    //�p��j, ex AtoB, BtoC, CtoD

	if (p.qRA == 0)
		SendTimes[SendTimesNum - 1][Type].CountType[CountType]++;    //�p��j, ex AtoB, BtoC, CtoD
	else if (p.qRA == nodeNum - 1)
		SendTimes[nodeNum - 2][Type].CountType[CountType]++;    //�p��j, ex AtoB, BtoC, CtoD
}
void Encode(Node &SendNode) {
	vector<Packet> outQ = SendNode.outputQueue;
	Packet p = outQ[0];
	Packet r;
	for (int i = 1; i < outQ.size(); i++) {
		if (outQ[i].RA == p.TA && outQ[i].TA == p.RA) {   //����
			r = p;
			r.cla = "NC";
			r.qseq = outQ[i].seq;
			r.qSA = outQ[i].SA;
			r.qDA = outQ[i].DA;
			r.qRA = outQ[i].RA;
			r.qThrowNum = outQ[i].ThrowNum;

			//cout<<"SendNode: "<<SendNode.number<<endl;
			/*cout << "NC" << endl;
			cout << r.type << "\t" << r.cla << "\t" << r.Duration << endl;
			cout << r.seq << "\t" << r.SA << "\t" << r.DA << "\t" << r.RA << "\t" << r.ThrowNum << "\t" << r.TA << endl;
			cout << r.qseq << "\t" << r.qSA << "\t" << r.qDA << "\t" << r.qRA << "\t" << r.qThrowNum << endl;

			cout << r.seq << " " << r.qseq << endl;
			cout << r.ThrowNum << " " << r.qThrowNum << endl;
			cout << p.ThrowNum << " " << outQ[i].ThrowNum << endl;*/

			if (p.RA != p.DA) {
				p.ThrowNum = 0;
				//cout << "push1" << endl;
				PushToQueue(SendNode.decodeQueue, p);
			}

			if (outQ[i].RA != outQ[i].DA) {
				outQ[i].ThrowNum = 0;
				//cout << "push2" << endl;
				PushToQueue(SendNode.decodeQueue, outQ[i]);
			}
			SendNode.outputQueue.erase(SendNode.outputQueue.begin() + i);
			SendNode.outputQueue[0] = r;
			//EncodeTimes++;
			break;
		}
	}
	if (r.cla != "NC") {    //�s�X���ѣ�
	}
}
void Decode(Node nodes[nodeNum], Node & SendNode, Packet & after, int receiveNode) {
	//cout << "Decode" << endl;
	Packet r = SendNode.outputQueue[0];
	for (int i = 0; i < nodes[receiveNode].decodeQueue.size(); i++) {
		if (nodes[receiveNode].decodeQueue[i].SA == r.SA && nodes[receiveNode].decodeQueue[i].DA == r.DA && receiveNode == r.qRA) { //�ѥX�ӬOq
			while (nodes[receiveNode].decodeQueue[i].ThrowNum > 0 && r.ThrowNum > 0) {
				r.ThrowNum--;
				nodes[receiveNode].decodeQueue[i].ThrowNum--;
			}
			if (r.ThrowNum > 0) {   //�~��R��decodeQueue���T��
				nodes[receiveNode].decodeQueue.erase(nodes[receiveNode].decodeQueue.begin() + i);
				i--;
				r.ThrowNum--;
			}
			else {
				if (nodes[receiveNode].decodeQueue[i].ThrowNum > 0) {   //�n�ѽX���Q�R���F
					if (r.RA == 0 || r.RA == nodeNum - 1) {
						DADecodeError++;
					}
					after = errorPkt();
				}
				else {
					after = DecodeResultP(r, nodes[receiveNode].decodeQueue[i], 'q');
					DecodeTimes++;
				}
				nodes[receiveNode].decodeQueue.erase(nodes[receiveNode].decodeQueue.begin() + i);
				return;
			}
		}
		else if (nodes[receiveNode].decodeQueue[i].SA == r.qSA && nodes[receiveNode].decodeQueue[i].DA == r.qDA && receiveNode == r.RA) {  //�ѥX�ӬOp
			while (nodes[receiveNode].decodeQueue[i].ThrowNum > 0 && r.qThrowNum > 0) {
				r.qThrowNum--;
				nodes[receiveNode].decodeQueue[i].ThrowNum--;
			}
			if (r.qThrowNum > 0) {   //�~��R��decodeQueue���T��
				nodes[receiveNode].decodeQueue.erase(nodes[receiveNode].decodeQueue.begin() + i);
				i--;
				r.qThrowNum--;
			}
			else {
				if (nodes[receiveNode].decodeQueue[i].ThrowNum > 0) {   //�n�ѽX���Q�R���F
					if (r.qRA == 0 || r.qRA == nodeNum - 1) {
						DADecodeError++;
					}
					after = errorPkt();
				}
				else {
					after = DecodeResultP(r, nodes[receiveNode].decodeQueue[i], 'p');
					DecodeTimes++;
				}
				nodes[receiveNode].decodeQueue.erase(nodes[receiveNode].decodeQueue.begin() + i);
				return;
			}
		}
	}
	after = errorPkt();
	if (r.RA == 0 || r.RA == nodeNum - 1) {
		DADecodeError++;
	}
}
void PushToQueue(vector <Packet> &Que, Packet pushPacket) {
	if (Que.size() >= queueSize) {
		bool isSame = false;
		for (int i = 0; i < Que.size(); i++) {
			for (int j = i + 1; j < Que.size(); j++) {
				if (Que[i].SA == Que[j].SA && Que[i].DA == Que[j].DA &&
					Que[i].TA == Que[j].TA && Que[i].RA == Que[j].RA) {   //�P�t�C�T��
					Que[j].ThrowNum = Que[i].ThrowNum + 1;
					Que.erase(Que.begin() + i);   //�R���ª�
					i--;
					isSame = true;
					break;
				}
			}
			if (isSame) break;
		}
		//cout << "full del" << endl;
	}
	Que.push_back(pushPacket);
}
Packet DecodeResultP(Packet r, Packet p, char pa) {
	Packet returnP = r;
	if (pa == 'p') {    //�ѥX�ӬOp
		if (returnP.SA == 0)
			returnP.cla = "AtoD";
		else
			returnP.cla = "DtoA";

		returnP.ThrowNum = 0;

		returnP.qseq = -1;
		returnP.qSA = -1;
		returnP.qDA = -1;
		returnP.qRA = -2;
		returnP.qThrowNum = 0;
	}
	else {   //�ѥX�ӬOq
		if (returnP.qSA == 0)
			returnP.cla = "AtoD";
		else
			returnP.cla = "DtoA";

		returnP.seq = returnP.qseq;
		returnP.SA = returnP.qSA;
		returnP.DA = returnP.qDA;
		returnP.RA = returnP.qRA;
		returnP.ThrowNum = 0;

		returnP.qseq = -1;
		returnP.qSA = -1;
		returnP.qDA = -1;
		returnP.qRA = -2;
		returnP.qThrowNum = 0;
	}
	return returnP;
}
void IntermediateNode(Packet & ReceiveP, Node nodes[nodeNum], vector<Node> &lists, vector <TimePoint> &listTime) {
	//cout << "intermidiate" << endl;
	int ReceiveNodeNum = ReceiveP.RA;
	if (ReceiveP.cla == "AtoD") {
		for (int i = 0; i < nodeNum - 1; i++)
			if (ReceiveP.RA == i) {
				ReceiveP.RA = i + 1;
				break;
			}
	}
	else if (ReceiveP.cla == "DtoA") {
		for (int i = nodeNum - 1; i > 0; i--)
			if (ReceiveP.RA == i) {
				ReceiveP.RA = i - 1;
				break;
			}
	}
	else {
		//NC �Ӳz�����i����
		cout << "Intermediate RA failed" << endl;
		cout << ReceiveP.type << "\t" << ReceiveP.cla << "\t" << ReceiveP.Duration << endl;
		cout << ReceiveP.seq << "\t" << ReceiveP.SA << "\t" << ReceiveP.DA << "\t" << ReceiveP.RA << "\t" << ReceiveP.ThrowNum << "\t" << ReceiveP.TA << endl;
		cout << ReceiveP.qseq << "\t" << ReceiveP.qSA << "\t" << ReceiveP.qDA << "\t" << ReceiveP.qRA << "\t" << ReceiveP.qThrowNum << endl;
	}
	if (ReceiveP.type == "Data" && ReceiveP.cla != "NC") {
		//cout << "intermediate " << ReceiveNodeNum << endl;
		Packet ack = add_ack(ReceiveP.TA);
		nodes[ReceiveNodeNum].outputQueue.insert(nodes[ReceiveNodeNum].outputQueue.begin(), ack);

		Node A = nodes[ReceiveNodeNum];
		A.backoffTime = 0;
		A.StartTimes = nodes[ReceiveP.TA].StartTimes - Wait_ACK_Time + transmission_delay_data + SIFS;
		lists.push_back(A);

		TimePoint TP;
		TP.backoff_T = A.backoffTime;
		TP.Send_N = ReceiveNodeNum;
		TP.Start_T = A.StartTimes;
		TP.DoWhat = "Send_Collision";
		listTime.push_back(TP);
	}
	else if (ReceiveP.type == "ACK") {
		nodes[ReceiveNodeNum].StartTimes = nodes[ReceiveNodeNum].StartTimes;	//����?
	}
	else {   //NC��ACK
		cout << "NC ACK" << endl;
	}
}
Packet add_p(int seq) {
	Packet pack;
	pack.type = "Data";
	pack.cla = "AtoD";
	pack.seq = seq;
	pack.Duration = -1;
	pack.TA = 0;
	pack.SA = 0;
	pack.DA = nodeNum - 1;
	pack.RA = 1; //����RA
	pack.ThrowNum = 0;
	pack.qseq = -1;
	pack.qSA = -1;
	pack.qDA = -1;
	pack.qRA = -2;
	pack.qThrowNum = 0;
	return pack;
}
Packet add_q(int seq) {
	Packet pack;
	pack.type = "Data";
	pack.cla = "DtoA";
	pack.seq = seq;
	pack.Duration = -1;
	pack.TA = nodeNum - 1;
	pack.SA = nodeNum - 1;
	pack.DA = 0;
	pack.RA = nodeNum - 2; //����RA
	pack.ThrowNum = 0;
	pack.qseq = -1;
	pack.qSA = -1;
	pack.qDA = -1;
	pack.qRA = -2;
	pack.qThrowNum = 0;
	return pack;
}
Packet add_ack(int RA) {
	Packet pack;
	pack.type = "ACK";
	pack.cla = "ACK";
	pack.Duration = 0;
	pack.seq = -1;
	pack.TA = -1;
	pack.SA = -1;
	pack.DA = -1;
	pack.RA = RA; //����RA
	pack.ThrowNum = 0;
	pack.qseq = -1;
	pack.qSA = -1;
	pack.qDA = -1;
	pack.qRA = -2;
	pack.qThrowNum = 0;
	return pack;
}
Packet errorPkt() {
	Packet err;

	err.type = "err";
	err.cla = "err";
	err.seq = -1;
	err.Duration = -1;
	err.SA = -1;
	err.DA = -1;
	err.RA = -1;
	err.TA = -1;
	err.ThrowNum = -1;

	err.qseq = -1;
	err.qSA = -1;
	err.qDA = -1;
	err.qRA = -1;
	err.qThrowNum = -1;
	return err;
}
bool cmp(Node a, Node b) {
	if (a.backoffTime < b.backoffTime)
		return true;
	else
		return false;
}
bool cmp2(TimePoint a, TimePoint b) {
	if (a.Start_T < b.Start_T)
		return true;
	else
		return false;
}
