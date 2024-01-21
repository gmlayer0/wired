#!/bin/python3
# We use a DAG to describe dataflow of controlling signals
# A -> B -> {C01->C02,C1,C2} -> D
# can be describe as follow
'''
node_relations = [
    ["C2": "D"],
    ["C1": "D"],
    ["C02": "D"],
    ["C01": "C02"],
    ["B": "C01"],
    ["B": "C1"],
    ["B": "C2"],
    ["A": "B"],
    ["Entry": "A"]
]
'''
# Each node will be generate with a structure, and each relation in json
# will generate a function for convert between structure.

class design_parser:
    def __init__(self):
        # directly read from file
        self.const_list = []
        self.signal_list = {}    # signal_name -> {length,stage,default_value,invalid_value}
        self.node_priv_signal = {} # node_name -> list[signal_name]
        self.inst_list = {}      # inst_name   -> {opcode,signal_name: value}
        self.node_relations = [] # list[dict(node_name:node:name)]
        # generate
        self.entry_node  = '' # node_name
        self.node_dag    = {} # node_name -> list[node_name]
        self.node_signal = {} # node_name -> list[signal_name]
    
    def parse_dict(self, dict_info):
        if dict_info.get('const') is not None:
            sub_obj = dict_info.get('const')
            for pair in sub_obj:
                self.const_list.append(sub_obj[pair])
        if dict_info.get('signal') is not None:
            sub_obj = dict_info.get('signal')
            for signal in sub_obj:
                self.signal_list[signal] = sub_obj[signal]
                self.node_priv_signal[sub_obj[signal]['stage']] = signal
        if dict_info.get('inst') is not None:
            sub_obj = dict_info.get('inst')
            for inst in sub_obj:
                self.inst_list[inst] = sub_obj[inst]
        if dict_info.get('node_relations') is not None:
            sub_obj = dict_info.get('node_relations')
            self.node_relations.append(sub_obj)
        return

    def gen_dag(self):
        for relation in self.node_relations:
            self.node_dag[relation[1]]    = []
        for relation in self.node_relations:
            if(relation[0] == 'Entry'):
                self.entry_node = relation[1]
                continue
            self.node_dag[relation[0]].append(relation[1])
            self.node_dag[relation[0]] += self.node_dag[relation[1]]
        for node in self.node_dag:
            node_list = self.node_dag[node]
            inst_list = []
            inst_list += self.node_priv_signal(node)
            for target in node_list:
                inst_list += self.node_priv_signal(target)
            self.node_signal = inst_list

    def gen_header(self):
        str_builder = "`ifndef _DECODE_HEADER\n`define _DECODE_HEADER\n\n"
        # const value define
        for const_value in self.const_list:
            str_builder += '`define '
            str_builder += const_value[0]
            str_builder += ' ('
            str_builder += const_value[1]
            str_builder += ')\n'
        str_builder += '\n'

        # signal struct define
        for leaf_struct in self.signal_list:
            str_builder += 'typedef logic ['
            str_builder += str(self.signal_list[leaf_struct][1] - 1)
            str_builder += ' : 0] '
            str_builder += leaf_struct
            str_builder += '_t;\n'
        str_builder += '\n'

        # node defination
        for node in self.node_signal:
            str_builder += 'typedef struct packed {\n'
            for signal_name in self.node_signal[node]:
                str_builder += '    ' + signal_name + '_t ' + signal_name + ';\n'
            str_builder += '} ' + node + '_t;\n\n'

        # convert function defination
        for relation in self.node_relations:
            from_stage = relation[0]
            to_stage   = relation[1]
            str_builder += 'function automatic ' + to_stage + '_t '
            # [name]([type] [value])
            str_builder += 'get_' + to_stage + '_from_' + from_stage + '(input ' + from_stage + '_t ' + from_stage + ');\n' 
            # <statement>
            str_builder += '    ' + to_stage + '_t ' + 'ret;\n'
            for signal_name in self.node_signal[to_stage]:
                str_builder += '    ret.' + signal_name + ' = ' + from_stage + '.' + signal_name + ';\n'
            str_builder += '    return ret;\n'
            # endfunction
            str_builder += 'endfunction\n\n'
        str_builder += "`endif\n"
        return str_builder

if __name__ == '__main__':
    parser = design_parser()
    parser.parse_dict()
    f = open('decoder.sv','w')
    f.write(parser.gen_sv_module())
    f.close()
    f = open('decoder.svh','w')
    f.write(parser.gen_sv_header())
    f.close()
    # -m : move decoder directly to rtl/decoder/
    if len(sys.argv) > 1:
        op = sys.argv[1]
        if op == '-m' or op == '--move':
            cur_file_path = os.path.split(os.path.realpath(__file__))[0]
            tar_path = os.path.join(cur_file_path, '../../rtl/decoder/')
            os.system("cp decoder.sv " + tar_path)
            print("cp decoder.sv " + tar_path)
            os.system("cp decoder.svh "+ tar_path)
            print("cp decoder.svh "+ tar_path)
