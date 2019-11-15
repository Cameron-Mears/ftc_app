package org.firstinspires.ftc.teamcode.util;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;

public class HashTable<Key,Val>
{
    private class Entry<K,V>
    {
        public K key;
        public V val;

        public Entry(K key,V val)
        {
            this.key = key;
            this.val = val;
        }
    }

    private ArrayList<LinkedList<Entry<Key,Val>>> table;
    private int capacity;

    public HashTable(int capacity)
    {
        this.capacity = capacity;
        this.table = new ArrayList<LinkedList<Entry<Key,Val>>>();

        for (int index = 0; index < capacity; index++)
        {
            table.add(new LinkedList<Entry<Key, Val>>());
        }

    }

    public Val get(Key key)
    {
           Iterator<Entry<Key,Val>> iter = table.get(key.hashCode() % capacity).iterator();
           Entry<Key,Val> entry;
           while (iter.hasNext())
           {
               entry = iter.next();
               if (entry.key.equals(key)) return entry.val;
           }
           return null;
    }

    public boolean put(Key key, Val val) throws KeyInUseException
    {
        LinkedList<Entry<Key,Val>> list = table.get(key.hashCode() % capacity);
        for (Entry<Key,Val> entry: list)
        {
            if (entry.key.equals(key))
            {
                throw new KeyInUseException(key.toString());
            }
        }
        return list.add(new Entry<Key,Val>(key,val));
    }



    public int getCapacity()
    {
        return capacity;
    }

}
